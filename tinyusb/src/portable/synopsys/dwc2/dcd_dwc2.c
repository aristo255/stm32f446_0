/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 William D. Jones
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jan Duempelmann
 * Copyright (c) 2020 Reinhard Panhuber
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && defined(TUP_USBIP_DWC2)

#if !(CFG_TUD_DWC2_SLAVE_ENABLE || CFG_TUD_DWC2_DMA_ENABLE)
#error DWC2 require either CFG_TUD_DWC2_SLAVE_ENABLE or CFG_TUD_DWC2_DMA_ENABLE to be enabled
#endif

// Debug level for DWC2
#define DWC2_DEBUG    2

#include "device/dcd.h"
#include "device/usbd_pvt.h"
#include "dwc2_common.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
typedef struct {
  uint8_t* buffer;
  tu_fifo_t* ff;
  uint16_t total_len;
  uint16_t max_size;
  uint8_t interval;
} xfer_ctl_t;

// This variable is modified from ISR context, so it must be protected by critical section
static xfer_ctl_t xfer_status[DWC2_EP_MAX][2];
#define XFER_CTL_BASE(_ep, _dir) (&xfer_status[_ep][_dir])

typedef struct {
  // EP0 transfers are limited to 1 packet - larger sizes has to be split
  uint16_t ep0_pending[2];  // Index determines direction as tusb_dir_t type
  uint16_t dfifo_top;      // top free location in DFIFO in words

  // Number of IN endpoints active
  uint8_t allocated_epin_count;

  // SOF enabling flag - required for SOF to not get disabled in ISR when SOF was enabled by
  bool sof_en;
} dcd_data_t;

static dcd_data_t _dcd_data;

volatile uint32_t dbg_usb_in_complete = 0;

CFG_TUD_MEM_SECTION static struct {
  TUD_EPBUF_DEF(setup_packet, 8);
} _dcd_usbbuf;

TU_ATTR_ALWAYS_INLINE static inline uint8_t dwc2_ep_count(const dwc2_regs_t* dwc2) {
  #if TU_CHECK_MCU(OPT_MCU_GD32VF103)
  return DWC2_EP_MAX;
  #else
  const dwc2_ghwcfg2_t ghwcfg2 = {.value = dwc2->ghwcfg2};
  return ghwcfg2.num_dev_ep + 1;
  #endif
}


//--------------------------------------------------------------------
// DMA
//--------------------------------------------------------------------
#if CFG_TUD_MEM_DCACHE_ENABLE
bool dcd_dcache_clean(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_clean(addr, data_size);
}

bool dcd_dcache_invalidate(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_invalidate(addr, data_size);
}

bool dcd_dcache_clean_invalidate(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_clean_invalidate(addr, data_size);
}
#endif

TU_ATTR_ALWAYS_INLINE static inline bool dma_device_enabled(const dwc2_regs_t* dwc2) {
  (void) dwc2;
  // Internal DMA only
  const dwc2_ghwcfg2_t ghwcfg2 = {.value = dwc2->ghwcfg2};
  return CFG_TUD_DWC2_DMA_ENABLE && ghwcfg2.arch == GHWCFG2_ARCH_INTERNAL_DMA;
}

static void dma_setup_prepare(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  if (dwc2->gsnpsid >= DWC2_CORE_REV_3_00a) {
    if(dwc2->epout[0].doepctl & DOEPCTL_EPENA) {
      return;
    }
  }

  // Receive only 1 packet
  dwc2->epout[0].doeptsiz = (1 << DOEPTSIZ_STUPCNT_Pos) | (1 << DOEPTSIZ_PKTCNT_Pos) | (8 << DOEPTSIZ_XFRSIZ_Pos);
  dwc2->epout[0].doepdma = (uintptr_t) _dcd_usbbuf.setup_packet;
  dwc2->epout[0].doepctl |= DOEPCTL_EPENA | DOEPCTL_USBAEP;
}

//--------------------------------------------------------------------+
// Data FIFO
//--------------------------------------------------------------------+


/* Схема организации FIFO для данных устройства

  FIFO разделено на следующие области:
  - EPInfo: для хранения метаданных DMA (требуется только при использовании DMA). Максимальный размер определяется как
    EP_LOC_CNT = ep_fifo_size - ghwcfg3.dfifo_depth. Если размер меньше EP_LOC_CNT, регистр gdfifocfg необходимо настроить до
    установки бита gahbcfg.dmaen.
      - Буферный режим: 1 слово на направление конечной точки (endpoint).
      - Режим Scatter/Gather DMA: 4 слова на направление конечной точки.
  - TX FIFO: отдельное FIFO для каждой конечной точки типа IN. Размер динамически изменяется в зависимости от размера пакета; распределение начинается с верхней границы памяти (начиная с EP0 IN).
  - Общее RX FIFO: единое FIFO для всех конечных точек типа OUT. Обычно вмещает до 2 пакетов максимального для конечной точки размера.

  Распределение TX FIFO выполняется от верхней границы памяти к нижней (с использованием верхнего указателя); это позволяет RX FIFO динамически увеличиваться в объеме,
  поскольку свободное пространство находится между областями RX FIFO и TX FIFO.

   ---------------- ep_fifo_size
  |  DxEPIDMAn  |
  |-------------|-- gdfifocfg.EPINFOBASE (max is ghwcfg3.dfifo_depth)
  | IN FIFO 0   |       control EP
  |-------------|
  | IN FIFO 1   |
  |-------------|
  |   . . . .   |
  |-------------|
  | IN FIFO n   |
  |-------------|
  |    FREE     |
  |-------------|-- GRXFSIZ (expandable)
  |  OUT FIFO   |
  | ( Shared )  |
  --------------- 0

  Согласно разделу «Выделение памяти FIFO» в техническом руководстве (RM), память FIFO распределяется следующим образом (размер слова — 32 бита):
- Для каждой конечной точки IN (EP IN) требуется объем памяти, как минимум равный максимальному размеру пакета.
- Все конечные точки OUT (EP OUT) совместно используют общую область OUT FIFO, объем которой рассчитывается так (для режимов Slave или Buffer DMA; для Scatter/Gather DMA используется иная формула):
  - 13 слов для пакетов SETUP и управляющих слов (до 3 пакетов SETUP).
  - 1 слово для глобального сигнала NAK (в данном случае не требуется/не используется).
  - Largest-EPsize/4 + 1 (FS: 64 байта, HS: 512 байт). Рекомендуемое значение: «2 x (Largest-EPsize/4 + 1)».
  - 2 слова для каждой используемой конечной точки OUT.

  Таким образом: GRXFSIZ = 13 + 1 + 2 x (Largest-EPsize/4 + 1) + 2 x EPOUTnum.
*/

TU_ATTR_ALWAYS_INLINE static inline uint16_t calc_device_grxfsiz(uint16_t largest_ep_size, uint8_t ep_count) {
  return 13 + 1 + 2 * ((largest_ep_size / 4) + 1) + 2 * ep_count;
}

static bool dfifo_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t packet_size) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const dwc2_controller_t* dwc2_controller = &_dwc2_controller[rhport];
  const uint8_t ep_count = dwc2_controller->ep_count;
  const uint8_t epnum = tu_edpt_number(ep_addr);
  const uint8_t dir = tu_edpt_dir(ep_addr);

  TU_ASSERT(epnum < ep_count);

  uint16_t fifo_size = tu_div_ceil(packet_size, 4);
  if (dir == TUSB_DIR_OUT) {
    //Рассчитайте требуемый размер RX FIFO.
    const uint16_t new_sz = calc_device_grxfsiz(4 * fifo_size, ep_count);

    // Если необходимо увеличить size_rx, проверьте, достаточно ли свободного места.
    if (dwc2->grxfsiz < new_sz) {
      TU_ASSERT(new_sz <= _dcd_data.dfifo_top);
      dwc2->grxfsiz = new_sz; // Enlarge RX FIFO
    }
  } else {
    // Проверка лимита одновременно активных конечных точек
    if(dwc2_controller->ep_in_count) {
      TU_ASSERT(_dcd_data.allocated_epin_count < dwc2_controller->ep_in_count);
      _dcd_data.allocated_epin_count++;
    }

    // Если TXFELVL настроен на режим «наполовину пуст» (half empty), то размер FIFO должен вдвое превышать max_size
    if ((dwc2->gahbcfg & GAHBCFG_TX_FIFO_EPMTY_LVL) == 0) {
      fifo_size *= 2;
    }

    // Проверьте наличие свободного места.
    TU_ASSERT(_dcd_data.dfifo_top >= fifo_size + dwc2->grxfsiz);
    _dcd_data.dfifo_top -= fifo_size;
    // TU_LOG(DWC2_DEBUG, "    TX FIFO %u: выделено %u слов со смещением %u\r\n", epnum, fifo_size, dfifo_top);

    // И TXFD, и TXSA измеряются в 32-битных словах..
    if (epnum == 0) {
      dwc2->dieptxf0 = (fifo_size << DIEPTXF0_TX0FD_Pos) | _dcd_data.dfifo_top;
    } else {
      // DIEPTXF начинается с FIFO № 1.
      dwc2->dieptxf[epnum - 1] = (fifo_size << DIEPTXF_INEPTXFD_Pos) | _dcd_data.dfifo_top;
    }
  }

  return true;
}

static void dfifo_device_init(uint8_t rhport) {
  const dwc2_controller_t* dwc2_controller = &_dwc2_controller[rhport];
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  dwc2->grxfsiz = calc_device_grxfsiz(CFG_TUD_ENDPOINT0_SIZE, dwc2_controller->ep_count);

  // Режим DMA Scatter/Gather пока не поддерживается. Для режима Buffer DMA требуется всего одно слово на направление конечной точки
  const bool is_dma = dma_device_enabled(dwc2);
  _dcd_data.dfifo_top = dwc2_controller->ep_fifo_size/4;
  if (is_dma) {
    _dcd_data.dfifo_top -= 2 * dwc2_controller->ep_count;
  }
  dwc2->gdfifocfg = (_dcd_data.dfifo_top << GDFIFOCFG_EPINFOBASE_SHIFT) | _dcd_data.dfifo_top;

  //Выделить FIFO для входа EP0
  dfifo_alloc(rhport, 0x80, CFG_TUD_ENDPOINT0_SIZE);
}


//--------------------------------------------------------------------
//  Конечная точка
//--------------------------------------------------------------------
static void edpt_activate(uint8_t rhport, const tusb_desc_endpoint_t* p_endpoint_desc) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint8_t epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  const uint8_t dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);

  xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, dir);
  xfer->max_size = tu_edpt_packet_size(p_endpoint_desc);
  xfer->interval = p_endpoint_desc->bInterval;

  // Контроль конечных точек
  dwc2_depctl_t depctl = {.value = 0};
  depctl.mps = xfer->max_size;
  depctl.active = 1;
  depctl.type = p_endpoint_desc->bmAttributes.xfer;
  if (p_endpoint_desc->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS) {
    depctl.set_data0_iso_even = 1;
  }
  if (dir == TUSB_DIR_IN) {
    depctl.tx_fifo_num = epnum;
  }

  dwc2_dep_t* dep = &dwc2->ep[dir == TUSB_DIR_IN ? 0 : 1][epnum];
  dep->ctl = depctl.value;
  dwc2->daintmsk |= TU_BIT(epnum + DAINT_SHIFT(dir));
}

static void edpt_disable(uint8_t rhport, uint8_t ep_addr, bool stall) {
  (void) rhport;

  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint8_t epnum = tu_edpt_number(ep_addr);
  const uint8_t dir = tu_edpt_dir(ep_addr);
  dwc2_dep_t* dep = &dwc2->ep[dir == TUSB_DIR_IN ? 0 : 1][epnum];

  if (dir == TUSB_DIR_IN) {
    // Отключить только активную в данный момент конечную точку, не являющуюся конечной точкой управления.
    if ((epnum == 0) || !(dep->diepctl & DIEPCTL_EPENA)) {
      dep->diepctl |= DIEPCTL_SNAK | (stall ? DIEPCTL_STALL : 0);
    } else {
      //Прекратить передачу пакетов и отвечать NAK на транзакции IN..
      dep->diepctl |= DIEPCTL_SNAK;
      while ((dep->diepint & DIEPINT_INEPNE) == 0) {}

      // Отключите конечную точку.
      dep->diepctl |= DIEPCTL_EPDIS | (stall ? DIEPCTL_STALL : 0);
      while ((dep->diepint & DIEPINT_EPDISD_Msk) == 0) {}

      dep->diepint = DIEPINT_EPDISD;
    }

    // Очистите FIFO и дождитесь подтверждения того, что он очищен.
    dfifo_flush_tx(dwc2, epnum);
  } else {
    //Отключить только активную в данный момент конечную точку, не являющуюся конечной точкой управления.
    if ((epnum == 0) || !(dep->doepctl & DOEPCTL_EPENA)) {
      dep->doepctl |= stall ? DOEPCTL_STALL : 0;
    } else {
      //  Для перевода OUT-конечной точки в состояние STALL необходимо установить бит GONAK.
        // Здесь проще использовать опрос (polling); мы всё равно не используем
        // прерывание "B"OUTNAKEFF, к тому же пользовательский код не может
        // его сбросить. Если этот цикл while никогда не завершится,
        // значит, у нас есть проблемы посерьезнее, чем просто стек.
      dwc2->dctl |= DCTL_SGONAK;
      while ((dwc2->gintsts & GINTSTS_BOUTNAKEFF_Msk) == 0) {}

      //  У меня то же самое — отключите эндпоинт..
      dep->doepctl |= DOEPCTL_EPDIS | (stall ? DOEPCTL_STALL : 0);
      while ((dep->doepint & DOEPINT_EPDISD_Msk) == 0) {}

      dep->doepint = DOEPINT_EPDISD;

      // Разрешите другим конечным точкам OUT продолжать получать данные.
      dwc2->dctl |= DCTL_CGONAK;
    }
  }
}

//Поскольку эта функция возвращает void, невозможно вернуть булев признак успешного выполнения.
// Необходимо гарантировать, что эта функция не будет вызвана, когда конечная точка (EP) отключена.
// Вызов должен осуществляться из критической секции
static void edpt_schedule_packets(uint8_t rhport, const uint8_t epnum, const uint8_t dir) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  xfer_ctl_t* const xfer = XFER_CTL_BASE(epnum, dir);
  dwc2_dep_t* dep = &dwc2->ep[dir == TUSB_DIR_IN ? 0 : 1][epnum];

  uint16_t num_packets;
  uint16_t total_bytes;

  // EP0 ограничен одним пакетом за передачу.
  if (epnum == 0) {
    total_bytes = tu_min16(_dcd_data.ep0_pending[dir], xfer->max_size);
    _dcd_data.ep0_pending[dir] -= total_bytes;
    num_packets = 1;
  } else {
    total_bytes = xfer->total_len;
    num_packets = tu_div_ceil(total_bytes, xfer->max_size);
    if (num_packets == 0) {
      num_packets = 1; //  Пакет нулевой длины по-прежнему считается за 1
    }
  }

  //Размер передачи: полная передача OUT (возможно, состоящая из нескольких пакетов) инициирует событие XFRC.
  dwc2_ep_tsize_t deptsiz = {.value = 0};
  deptsiz.xfer_size = total_bytes;
  deptsiz.packet_count = num_packets;
  dep->tsiz = deptsiz.value;

  //контроль
  dwc2_depctl_t depctl = {.value = dep->ctl};
  depctl.clear_nak = 1;
  depctl.enable = 1;
  if (depctl.type == DEPCTL_EPTYPE_ISOCHRONOUS && xfer->interval == 1) {
    const dwc2_dsts_t dsts = {.value = dwc2->dsts};
    const uint32_t odd_now = dsts.frame_number & 1u;
    if (odd_now) {
      depctl.set_data0_iso_even = 1;
    } else {
      depctl.set_data1_iso_odd = 1;
    }
  }

  const bool is_dma = dma_device_enabled(dwc2);
  if(is_dma) {
    if (dir == TUSB_DIR_IN && total_bytes != 0) {
      dcd_dcache_clean(xfer->buffer, total_bytes);
    }
    dep->diepdma = (uintptr_t) xfer->buffer;
    dep->diepctl = depctl.value; // включить конечную точку
  } else {
    dep->diepctl = depctl.value; // включить конечную точку

    // Разрешайте прерывание по опустошению FIFO передачи (TX FIFO) только при наличии данных. Обратите внимание: это необходимо делать после включения бита в регистре depctl.
    if (dir == TUSB_DIR_IN && total_bytes != 0) {
      dwc2->diepempmsk |= (1 << epnum);
    }
  }
}

//--------------------------------------------------------------------
//API контроллера
//--------------------------------------------------------------------
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  tu_memclr(&_dcd_data, sizeof(_dcd_data));

  //  Инициализация ядра
  const bool is_highspeed = dwc2_core_is_highspeed(dwc2, TUSB_ROLE_DEVICE);
  const bool is_dma = dma_device_enabled(dwc2);
  TU_ASSERT(dwc2_core_init(rhport, is_highspeed, is_dma));

  //------------- 7.1 Инициализация устройства -------------//
  // Установить максимальную скорость устройства
  uint32_t dcfg = dwc2->dcfg & ~DCFG_DSPD_Msk;
  if (is_highspeed) {
    dcfg |= DCFG_DSPD_HS << DCFG_DSPD_Pos;

    // XCVRDLY: требуется задержка приемопередатчика между сигналами xcvr_sel и txvalid во время выполнения процедуры chirp устройством
    // при использовании с некоторыми PHY, такими как USB334x (USB3341, USB3343, USB3346, USB3347)
    const dwc2_ghwcfg2_t ghwcfg2 = {.value = dwc2->ghwcfg2};
    if (ghwcfg2.hs_phy_type == GHWCFG2_HSPHY_ULPI) {
      dcfg |= DCFG_XCVRDLY;
    }
  } else {
    dcfg |= DCFG_DSPD_FS << DCFG_DSPD_Pos;
  }

  dcfg |= DCFG_NZLSOHSK; // Отправить STALL и отбросить пакет, если хост отправляет данные, отличные от ZLP, на этапе статуса управляющей передачи.
  dwc2->dcfg = dcfg;

  dcd_disconnect(rhport);

  // Принудительный режим устройства
  dwc2->gusbcfg = (dwc2->gusbcfg & ~GUSBCFG_FHMOD) | GUSBCFG_FDMOD;

  // Сбросить блокировку A, принудительно установить признак достоверности для B
  dwc2->gotgctl = (dwc2->gotgctl & ~GOTGCTL_AVALOEN) | GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL;

#if CFG_TUSB_MCU == OPT_MCU_STM32N6
  //В STM32N6 отсутствует аппаратное обнаружение Vbus B-session.
  dwc2->stm32_gccfg |= STM32_GCCFG_VBVALOVAL;
#endif

  // Включить необходимые прерывания
  dwc2->gintmsk |= GINTMSK_OTGINT | GINTMSK_USBRST | GINTMSK_ENUMDNEM | GINTMSK_WUIM;

  // Уровень пустости TX FIFO для прерывания завершен.
  uint32_t gahbcfg = dwc2->gahbcfg;
  gahbcfg |= GAHBCFG_TX_FIFO_EPMTY_LVL;
  gahbcfg |= GAHBCFG_GINT; // Разрешить глобальные прерывания
  dwc2->gahbcfg = gahbcfg;

  dcd_connect(rhport);
  return true;
}

void dcd_int_enable(uint8_t rhport) {
  dwc2_dcd_int_enable(rhport);
}

void dcd_int_disable(uint8_t rhport) {
  dwc2_dcd_int_disable(rhport);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  dwc2->dcfg = (dwc2->dcfg & ~DCFG_DAD_Msk) | (dev_addr << DCFG_DAD_Pos);

  //Ответ со статусом после изменения адреса устройства
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport) {
  (void) rhport;

  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  // настроить удаленное пробуждение
  dwc2->dctl |= DCTL_RWUSIG;

  //разрешить SOF обнаруживать возобновление активности шины
  dwc2->gintsts = GINTSTS_SOF;
  dwc2->gintmsk |= GINTMSK_SOFM;

  // Согласно спецификации: бит сигнала удаленного пробуждения должен быть сброшен в течение 1–15 мс.
  dwc2_remote_wakeup_delay();

  dwc2->dctl &= ~DCTL_RWUSIG;
}

void dcd_connect(uint8_t rhport) {
  (void) rhport;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

#ifdef TUP_USBIP_DWC2_ESP32
  usb_wrap_otg_conf_reg_t conf = USB_WRAP.otg_conf;
  conf.pad_pull_override = 0;
  conf.dp_pullup = 0;
  conf.dp_pulldown = 0;
  conf.dm_pullup = 0;
  conf.dm_pulldown = 0;
  USB_WRAP.otg_conf = conf;
#endif

  dwc2->dctl &= ~DCTL_SDIS;
}

void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

#ifdef TUP_USBIP_DWC2_ESP32
  usb_wrap_otg_conf_reg_t conf = USB_WRAP.otg_conf;
  conf.pad_pull_override = 1;
  conf.dp_pullup = 0;
  conf.dp_pulldown = 1;
  conf.dm_pullup = 0;
  conf.dm_pulldown = 1;
  USB_WRAP.otg_conf = conf;
#endif

  dwc2->dctl |= DCTL_SDIS;
}

//  Обратите внимание: классы для изохронных конечных точек (аудио, видео и, возможно, другие) намеренно используют функцию `dcd_sof_enable()`
// для включения или отключения соответствующего обработчика прерывания (ISR).

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  _dcd_data.sof_en = en;

  if (en) {
    dwc2->gintsts = GINTSTS_SOF;
    dwc2->gintmsk |= GINTMSK_SOFM;
  } else {
    dwc2->gintmsk &= ~GINTMSK_SOFM;
  }
}

/*------------------------------------------------------------------*/
/* Порт конечной точки DCD
 *------------------------------------------------------------------*/

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_edpt) {
  TU_ASSERT(dfifo_alloc(rhport, desc_edpt->bEndpointAddress, tu_edpt_packet_size(desc_edpt)));
  edpt_activate(rhport, desc_edpt);
  return true;
}

// Закройте все конечные точки, кроме управляющей, и отмените все ожидающие передачи (при их наличии).
void dcd_edpt_close_all(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  uint8_t const ep_count = _dwc2_controller[rhport].ep_count;

  usbd_spin_lock(false);

  _dcd_data.allocated_epin_count = 0;

  // Отключить немаскируемое прерывание
  dwc2->daintmsk = (1 << DAINTMSK_OEPM_Pos) | (1 << DAINTMSK_IEPM_Pos);

  for (uint8_t n = 1; n < ep_count; n++) {
    for (uint8_t d = 0; d < 2; d++) {
      dwc2_dep_t* dep = &dwc2->ep[d][n];
      if (dep->ctl & EPCTL_EPENA) {
        dep->ctl |= EPCTL_SNAK | EPCTL_EPDIS;
      }
      xfer_status[n][1-d].max_size = 0;
    }
  }

  dfifo_flush_tx(dwc2, 0x10); //все FIFO передачи
  dfifo_flush_rx(dwc2);
  dfifo_device_init(rhport); // повторная инициализация dfifo

  usbd_spin_unlock(false);
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  TU_ASSERT(dfifo_alloc(rhport, ep_addr, largest_packet_size));
  return true;
}

bool dcd_edpt_iso_activate(uint8_t rhport,  tusb_desc_endpoint_t const * p_endpoint_desc) {
  // Отключите конечную точку (EP), чтобы очистить потенциально незавершенные передачи.
  edpt_disable(rhport, p_endpoint_desc->bEndpointAddress, false);
  edpt_activate(rhport, p_endpoint_desc);
  return true;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, dir);
  bool ret;

  usbd_spin_lock(false);

  if (xfer->max_size == 0) {
    ret = false;  //  Конечная точка закрыта
  } else {
    xfer->buffer = buffer;
    xfer->ff = NULL;
    xfer->total_len = total_bytes;

    //EP0 может обрабатывать только один пакет.
    if (epnum == 0) {
      _dcd_data.ep0_pending[dir] = total_bytes;
    }

    // Планирование отправки пакетов в пределах интервала прерывания.
    edpt_schedule_packets(rhport, epnum, dir);
    ret = true;
  }

  usbd_spin_unlock(false);

  return ret;
}

//  Количество байтов необходимо указывать явно: во-первых, это обеспечивает более гибкое управление объемом
// записываемых данных, а во-вторых, позволяет использовать возвращаемое значение для передачи логического признака
// успешного выполнения операции. Если значение total_bytes превышает доступный объем данных, FIFO скопирует в USB-буфер
// только те данные, которые имеются в наличии.
bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t* ff, uint16_t total_bytes) {
  // USB-буферы всегда работают с байтами, поэтому во избежание лишних операций деления мы требуем, чтобы item_size был равен 1.
  TU_ASSERT(ff->item_size == 1);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, dir);
  bool ret;

  usbd_spin_lock(false);

  if (xfer->max_size == 0) {
    ret = false;  //Конечная точка закрыта
  } else {
    xfer->buffer = NULL;
    xfer->ff = ff;
    xfer->total_len = total_bytes;

    // Запланировать отправку пакетов в обработчике прерывания
    // TODO FIFO передачи может быть доступно только в режиме ведомого (slave mode)
    edpt_schedule_packets(rhport, epnum, dir);
    ret = true;
  }

  usbd_spin_unlock(false);

  return ret;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  edpt_disable(rhport, ep_addr, true);
  if((tu_edpt_number(ep_addr) == 0) && dma_device_enabled(dwc2)) {
    dma_setup_prepare(rhport);
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  dwc2_dep_t* dep = &dwc2->ep[dir == TUSB_DIR_IN ? 0 : 1][epnum];

  // Очистить состояние зависания и сбросить переключатель данных
  dep->ctl &= ~EPCTL_STALL;;
  dep->ctl |= EPCTL_SD0PID_SEVNFRM;
}

//--------------------------------------------------------------------
// Обработчик прерываний
//--------------------------------------------------------------------

// 7.4.1 Инициализация при сбросе USB
// Должна вызываться из критической секции
static void handle_bus_reset(uint8_t rhport) {
  dwc2_regs_t *dwc2 = DWC2_REG(rhport);
  const uint8_t ep_count =  dwc2_ep_count(dwc2);

  tu_memclr(xfer_status, sizeof(xfer_status));

  _dcd_data.sof_en = false;
  _dcd_data.allocated_epin_count = 0;

  // 1.NAK для всех выходных (OUT) конечных точек
  for (uint8_t n = 0; n < ep_count; n++) {
    dwc2->epout[n].doepctl |= DOEPCTL_SNAK;
  }

  // Отключить все конечные точки типа IN
  for (uint8_t n = 0; n < ep_count; n++) {
    if (dwc2->epin[n].diepctl & DIEPCTL_EPENA) {
      dwc2->epin[n].diepctl |= DIEPCTL_SNAK | DIEPCTL_EPDIS;
    }
  }

  // 2. Настроить маску прерываний для EP0
  dwc2->daintmsk = TU_BIT(DAINTMSK_OEPM_Pos) | TU_BIT(DAINTMSK_IEPM_Pos);
  dwc2->doepmsk = DOEPMSK_STUPM | DOEPMSK_XFRCM;
  dwc2->diepmsk = DIEPMSK_TOM | DIEPMSK_XFRCM;

  // 4. Настройте DFIFO
  dfifo_flush_tx(dwc2, 0x10); // all tx fifo
  dfifo_flush_rx(dwc2);
  dfifo_device_init(rhport);

  // 5. Сбросить адрес устройства
  dwc2_dcfg_t dcfg = {.value = dwc2->dcfg};
  dcfg.address = 0;
  dwc2->dcfg = dcfg.value;

  // Размер управляющей конечной точки EP0 для обоих случаев зафиксирован на уровне 64 байт.
  dwc2->epin[0].ctl &= ~(0x03 << DIEPCTL_MPSIZ_Pos);
  dwc2->epout[0].ctl &= ~(0x03 << DOEPCTL_MPSIZ_Pos);

  xfer_status[0][TUSB_DIR_OUT].max_size = 64;
  xfer_status[0][TUSB_DIR_IN].max_size = 64;

  if(dma_device_enabled(dwc2)) {
    dma_setup_prepare(rhport);
  } else {
    dwc2->epout[0].doeptsiz |= (3 << DOEPTSIZ_STUPCNT_Pos);
  }

  dwc2->gintmsk |= GINTMSK_OEPINT | GINTMSK_IEPINT;
}

static void handle_enum_done(uint8_t rhport) {
  dwc2_regs_t *dwc2 = DWC2_REG(rhport);
  const dwc2_dsts_t dsts = {.value = dwc2->dsts};
  tusb_speed_t speed;
  switch (dsts.enum_speed) {
    case DCFG_SPEED_HIGH:
      speed = TUSB_SPEED_HIGH;
    break;

    case DCFG_SPEED_LOW:
      speed = TUSB_SPEED_LOW;
    break;

    case DCFG_SPEED_FULL_30_60MHZ:
    case DCFG_SPEED_FULL_48MHZ:
    default:
      speed = TUSB_SPEED_FULL;
    break;
  }

  // TODO: необходимо обновить GUSBCFG_TRDT в соответствии со скоростью соединения..
  dcd_event_bus_reset(rhport, speed, true);
}

#if 0
TU_ATTR_ALWAYS_INLINE static inline void print_doepint(uint32_t doepint) {
  const char* str[] = {
    "XFRC", "DIS", "AHBERR", "SETUP_DONE",
    "ORXED", "STATUS_RX", "SETUP_B2B", "RSV7",
    "OPERR", "BNA", "RSV10", "ISODROP",
    "BBLERR", "NAK", "NYET", "SETUP_RX"
  };

  for(uint32_t i=0; i<TU_ARRAY_SIZE(str); i++) {
    if (doepint & TU_BIT(i)) {
      TU_LOG1("%s ", str[i]);
    }
  }
  TU_LOG1("\r\n");
}
#endif

#if CFG_TUD_DWC2_SLAVE_ENABLE
//  Обработка разделяемой FIFO-очереди приема; это прерывание используется только в режиме ведомого устройства (Slave mode).
static void handle_rxflvl_irq(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const volatile uint32_t* rx_fifo = dwc2->fifo[0];

  // Извлечь управляющее слово из FIFO
  const dwc2_grxstsp_t grxstsp = {.value = dwc2->grxstsp};
  const uint8_t epnum = grxstsp.ep_ch_num;

  dwc2_dep_t* epout = &dwc2->epout[epnum];

  switch (grxstsp.packet_status) {
    case GRXSTS_PKTSTS_GLOBAL_OUT_NAK:
      // Глобальный OUT NAK: ничего не делать
      break;

    case GRXSTS_PKTSTS_SETUP_RX: {
      //Получен пакет настройки
      uint32_t* setup = (uint32_t*)(uintptr_t) _dcd_usbbuf.setup_packet;
      //Мы можем получить до трех пакетов настройки подряд, но действительным является только последний из них..
      setup[0] = (*rx_fifo);
      setup[1] = (*rx_fifo);
      break;
    }

    case GRXSTS_PKTSTS_SETUP_DONE:
      // Обработка пакета SETUP завершена:
        // После извлечения пакета контроллер dwc2 инициирует прерывание DOEPINT_SETUP, которое обрабатывается функцией handle_epout_irq()
      epout->doeptsiz |= (3 << DOEPTSIZ_STUPCNT_Pos);
      break;

    case GRXSTS_PKTSTS_RX_DATA: {
      // Получен исходящий пакет
      const uint16_t byte_count = grxstsp.byte_count;
      xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);

      if (byte_count) {
        // Считать пакет из RxFIFO
        if (xfer->ff) {
          tu_fifo_write_n_const_addr_full_words(xfer->ff, (const void*) (uintptr_t) rx_fifo, byte_count);
        } else {
          dfifo_read_packet(dwc2, xfer->buffer, byte_count);
          xfer->buffer += byte_count;
        }

        // короткий пакет за вычетом оставшихся байтов (xfer_size)
        if (byte_count < xfer->max_size) {
          const dwc2_ep_tsize_t tsiz = {.value = epout->tsiz};
          xfer->total_len -= tsiz.xfer_size;
          if (epnum == 0) {
            xfer->total_len -= _dcd_data.ep0_pending[TUSB_DIR_OUT];
            _dcd_data.ep0_pending[TUSB_DIR_OUT] = 0;
          }
        }
      }
      break;
    }

    case GRXSTS_PKTSTS_RX_COMPLETE:
      // Обработка OUT-пакета завершена
        // После того как эта запись будет извлечена из FIFO приема, контроллер dwc2 инициирует прерывание
        // «Передача завершена» (Transfer Completed) для указанной OUT-конечной точки;
        // это прерывание будет обработано функцией handle_epout_irq()
      break;

    default: break;
  }
}

static void handle_epout_slave(uint8_t rhport, uint8_t epnum, dwc2_doepint_t doepint_bm) {

	if (epnum == 2)
	{
	    dbg_usb_in_complete++;
	}

  if (doepint_bm.setup_phase_done) {
    dcd_event_setup_received(rhport, _dcd_usbbuf.setup_packet, true);
    return;
  }

  // Обычная передача на выход завершена
  if (doepint_bm.xfer_complete) {
    //  Обрабатывать пропуск данных только в том случае, если это связано с фазой SETUP или статусом.
	    // Примечание: хотя флаги (xfer_complete + status_phase_rx) относятся только к режиму buffered DMA,
	    // в контроллере STM32L47x (dwc2 v3.00a) они могут устанавливаться при извлечении из очереди
	    // записи GRXSTS_PKTSTS_SETUP_RX; следовательно, они могут быть установлены до или одновременно
	    // с флагом setup_phase_done.
    if (!doepint_bm.status_phase_rx && !doepint_bm.setup_packet_rx) {
      xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);

      if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_OUT]) {
        // EP0 может обработать только один пакет; запланируйте прием следующего пакета.
        edpt_schedule_packets(rhport, epnum, TUSB_DIR_OUT);
      } else {
        dcd_event_xfer_complete(rhport, epnum, xfer->total_len, XFER_RESULT_SUCCESS, true);
      }
    }
  }
}

static void handle_epin_slave(uint8_t rhport, uint8_t epnum, dwc2_diepint_t diepint_bm) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  dwc2_dep_t* epin = &dwc2->epin[epnum];
  xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, TUSB_DIR_IN);

  if (diepint_bm.xfer_complete) {
    if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_IN]) {
      //EP0 может обрабатывать только один пакет. Запланируйте передачу еще одного пакета.
      edpt_schedule_packets(rhport, epnum, TUSB_DIR_IN);
    } else {
      dcd_event_xfer_complete(rhport, epnum | TUSB_DIR_IN_MASK, xfer->total_len, XFER_RESULT_SUCCESS, true);
    }
  }

  // Бит пустого состояния TX FIFO доступен только для чтения. Аппаратное обеспечение сбрасывает его только в том случае, если количество записанных байтов превышает:
  // - 64 байта или
  // - половину размера TX FIFO (настраивается через GAHBCFG.TXFELVL)
  if (diepint_bm.txfifo_empty && (dwc2->diepempmsk & (1 << epnum))) {
    dwc2_ep_tsize_t tsiz = {.value = epin->tsiz};
    const uint16_t remain_packets = tsiz.packet_count;

    // Обрабатывайте каждый пакет (в FIFO можно записывать только целые пакеты).
    for (uint16_t i = 0; i < remain_packets; i++) {
      tsiz.value = epin->tsiz;
      const uint16_t remain_bytes = (uint16_t) tsiz.xfer_size;
      const uint16_t xact_bytes = tu_min16(remain_bytes, xfer->max_size);

      // Проверьте, достаточно ли свободного места в dtxfsts.
      if (xact_bytes > ((epin->dtxfsts & DTXFSTS_INEPTFSAV_Msk) << 2)) {
        break;
      }

      // Отправить пакет в Tx-FIFO
      if (xfer->ff) {
        volatile uint32_t* tx_fifo = dwc2->fifo[epnum];
        tu_fifo_read_n_const_addr_full_words(xfer->ff, (void*)(uintptr_t)tx_fifo, xact_bytes);
      } else {
        dfifo_write_packet(dwc2, epnum, xfer->buffer, xact_bytes);
        xfer->buffer += xact_bytes;
      }
    }

    //Отключите TXFE, если записаны все байты..
    tsiz.value = epin->tsiz;
    if (tsiz.xfer_size == 0) {
      dwc2->diepempmsk &= ~(1 << epnum);
    }
  }
}
#endif

#if CFG_TUD_DWC2_DMA_ENABLE
static void handle_epout_dma(uint8_t rhport, uint8_t epnum, dwc2_doepint_t doepint_bm) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  if (doepint_bm.setup_phase_done) {
    dma_setup_prepare(rhport);
    dcd_dcache_invalidate(_dcd_usbbuf.setup_packet, 8);
    dcd_event_setup_received(rhport, _dcd_usbbuf.setup_packet, true);
    return;
  }

  // Внешняя передача завершена
  if (doepint_bm.xfer_complete) {
    // Обрабатывать пропуск данных только в том случае, если это связано с настройкой или состоянием
	    // Завершение обычной передачи OUT
    if (!doepint_bm.status_phase_rx && !doepint_bm.setup_packet_rx) {
      if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_OUT]) {
        // EP0 может обработать только один пакет; запланируйте прием следующего пакета..
        edpt_schedule_packets(rhport, epnum, TUSB_DIR_OUT);
      } else {
        dwc2_dep_t* epout = &dwc2->epout[epnum];
        xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);

        // определить фактическое количество полученных байтов
        const dwc2_ep_tsize_t tsiz = {.value = epout->tsiz};
        const uint16_t remain = tsiz.xfer_size;
        xfer->total_len -= remain;

        // Это ZLP, поэтому подготовьте EP0 к следующему этапу настройки
        // TODO: использовать прием на фазе статуса
        if(epnum == 0 && xfer->total_len == 0) {
          dma_setup_prepare(rhport);
        }

        dcd_dcache_invalidate(xfer->buffer, xfer->total_len);
        dcd_event_xfer_complete(rhport, epnum, xfer->total_len, XFER_RESULT_SUCCESS, true);
      }
    }
  }
}

static void handle_epin_dma(uint8_t rhport, uint8_t epnum, dwc2_diepint_t diepint_bm) {
  xfer_ctl_t* xfer = XFER_CTL_BASE(epnum, TUSB_DIR_IN);

  if (diepint_bm.xfer_complete) {
    if ((epnum == 0) && _dcd_data.ep0_pending[TUSB_DIR_IN]) {
      // EP0 может обрабатывать только один пакет. Запланируйте передачу еще одного пакета.
      edpt_schedule_packets(rhport, epnum, TUSB_DIR_IN);
    } else {
      if(epnum == 0) {
        dma_setup_prepare(rhport);
      }
      dcd_event_xfer_complete(rhport, epnum | TUSB_DIR_IN_MASK, xfer->total_len, XFER_RESULT_SUCCESS, true);
    }
  }
}
#endif

static void handle_ep_irq(uint8_t rhport, uint8_t dir) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const bool is_dma = dma_device_enabled(dwc2);
  const uint8_t ep_count = dwc2_ep_count(dwc2);
  const uint8_t daint_offset = (dir == TUSB_DIR_IN) ? DAINT_IEPINT_Pos : DAINT_OEPINT_Pos;
  dwc2_dep_t* ep_base = &dwc2->ep[dir == TUSB_DIR_IN ? 0 : 1][0];

  // Бит DAINT для заданной конечной точки (EP) сбрасывается при сбросе бита DEPINTx.
  // Бит EPINT сбрасывается при сбросе битов DAINT
  for (uint8_t epnum = 0; epnum < ep_count; epnum++) {
    if (dwc2->daint & TU_BIT(daint_offset + epnum)) {
      dwc2_dep_t* epout = &ep_base[epnum];
      union {
        uint32_t value;
        dwc2_diepint_t diepint_bm;
        dwc2_doepint_t doepint_bm;
      } intr;
      intr.value = epout->intr;

      epout->intr = intr.value; // Сброс прерывания

      if (is_dma) {
        #if CFG_TUD_DWC2_DMA_ENABLE
        if (dir == TUSB_DIR_IN) {
          handle_epin_dma(rhport, epnum, intr.diepint_bm);
        } else {
          handle_epout_dma(rhport, epnum, intr.doepint_bm);
        }
        #endif
      } else {
        #if CFG_TUD_DWC2_SLAVE_ENABLE
        if (dir == TUSB_DIR_IN) {
          handle_epin_slave(rhport, epnum, intr.diepint_bm);
        } else {
          handle_epout_slave(rhport, epnum, intr.doepint_bm);
        }
        #endif
      }
    }
  }
}

/*Иерархия прерываний
                 DIEPINT  DIEPINT
                    \       /
                     \     /
                      DAINT
                     /     \
                    /       \
     GINTSTS:    OEPInt    IEPInt | USBReset | EnumDone | USBSusp | WkUpInt | OTGInt | SOF | RXFLVL

  Примечание: при установке OTG_MULTI_PROC_INTRPT = 1 прерывания отдельных конечных точек (deachint/deachmsk/diepeachmsk/doepeachmsk)
  объединяются для формирования выделенной линии прерывания для каждой конечной точки.
 */
void dcd_int_handler(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint32_t gintmask = dwc2->gintmsk;
  const uint32_t gintsts = dwc2->gintsts & gintmask;

  if (gintsts & GINTSTS_USBRST) {
    // USBRST — это начало сброса
    dwc2->gintsts = GINTSTS_USBRST;

    usbd_spin_lock(true);
    handle_bus_reset(rhport);
    usbd_spin_unlock(true);
  }

  if (gintsts & GINTSTS_ENUMDNE) {
    // ENUMDNE — это завершение этапа сброса, на котором определяется скорость соединения.
    dwc2->gintsts = GINTSTS_ENUMDNE;
    dwc2->gintmsk |= GINTMSK_USBSUSPM;
    handle_enum_done(rhport);
  }

  if (gintsts & GINTSTS_USBSUSP) {
    dwc2->gintsts = GINTSTS_USBSUSP;
    dwc2->gintmsk &= ~GINTMSK_USBSUSPM;
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }

  if (gintsts & GINTSTS_WKUINT) {
    dwc2->gintsts = GINTSTS_WKUINT;
    dwc2->gintmsk |= GINTMSK_USBSUSPM;
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }

  // TODO проверить GINTSTS_DISCINT на предмет обнаружения разрыва соединения
  // if(int_status & GINTSTS_DISCINT)

  if (gintsts & GINTSTS_OTGINT) {
    //  Бит OTG INT доступен только для чтения.
    const uint32_t otg_int = dwc2->gotgint;

    if (otg_int & GOTGINT_SEDET) {
      dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
    }

    dwc2->gotgint = otg_int;
  }

  if(gintsts & GINTSTS_SOF) {
    dwc2->gintsts = GINTSTS_SOF;
    dwc2->gintmsk |= GINTMSK_USBSUSPM;
    const uint32_t frame = (dwc2->dsts & DSTS_FNSOF) >> DSTS_FNSOF_Pos;

    // Отключить прерывание SOF, если оно не было явно разрешено (поскольку SOF использовался для обнаружения удаленного пробуждения).
    if (!_dcd_data.sof_en) {
      dwc2->gintmsk &= ~GINTMSK_SOFM;
    }

    dcd_event_sof(rhport, frame, true);
  }

#if CFG_TUD_DWC2_SLAVE_ENABLE
  //  Обработка прерывания по заполнению RxFIFO.
  if (gintsts & GINTSTS_RXFLVL) {
    //Бит RXFLVL доступен только для чтения.
    dwc2->gintmsk &= ~GINTMSK_RXFLVLM; // отключить прерывание RXFLVL во время чтения

    do {
      handle_rxflvl_irq(rhport); // read all packets
    } while(dwc2->gintsts & GINTSTS_RXFLVL);

    dwc2->gintmsk |= GINTMSK_RXFLVLM;
  }
#endif

  // Обработка прерываний от OUT-конечной точки
  if (gintsts & GINTSTS_OEPINT) {
    // OEPINT доступен только для чтения; очистка выполняется с помощью DOEPINTn.
    handle_ep_irq(rhport, TUSB_DIR_OUT);
  }

  // Обработка прерываний от IN-конечной точки.
  if (gintsts & GINTSTS_IEPINT) {
    // Бит IEPINT доступен только для чтения; сброс осуществляется с помощью DIEPINTn
    handle_ep_irq(rhport, TUSB_DIR_IN);
  }
}

#if CFG_TUD_TEST_MODE
void dcd_enter_test_mode(uint8_t rhport, tusb_feature_test_mode_t test_selector) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  // Включите тестовый режим
  dwc2->dctl = (dwc2->dctl & ~DCTL_TCTL_Msk) | (((uint8_t) test_selector) << DCTL_TCTL_Pos);
}
#endif

#endif
