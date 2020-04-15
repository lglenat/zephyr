/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_RTC_0_NAME				DT_LABEL(DT_INST(0, nxp_imx_gpt))
#define DT_RTC_1_NAME				DT_LABEL(DT_INST(1, nxp_imx_gpt))

#define DT_ETH_MCUX_0_NAME			DT_NXP_KINETIS_ETHERNET_402D8000_LABEL
#define DT_ETH_MCUX_0_MAC			DT_NXP_KINETIS_ETHERNET_402D8000_LOCAL_MAC_ADDRESS
#define DT_ETH_MCUX_0_IRQ_PRI			DT_NXP_KINETIS_ETHERNET_402D8000_IRQ_0_PRIORITY
#define DT_IRQ_ETH_COMMON			DT_NXP_KINETIS_ETHERNET_402D8000_IRQ_0
#define DT_IRQ_ETH_IEEE1588_TMR			DT_NXP_KINETIS_PTP_402D8000_PTP_IRQ_0

#define DT_ETH_MCUX_1_NAME			DT_NXP_KINETIS_ETHERNET_402D4000_LABEL
#define DT_ETH_MCUX_1_MAC			DT_NXP_KINETIS_ETHERNET_402D4000_LOCAL_MAC_ADDRESS
#define DT_ETH_MCUX_1_IRQ_PRI			DT_NXP_KINETIS_ETHERNET_402D4000_IRQ_0_PRIORITY
#define DT_IRQ_ETH1_COMMON			DT_NXP_KINETIS_ETHERNET_402D4000_IRQ_0
#define DT_IRQ_ETH1_IEEE1588_TMR		DT_NXP_KINETIS_PTP_402D4000_PTP_IRQ_0

#define DT_VIDEO_MCUX_CSI_BASE_ADDRESS		DT_NXP_IMX_CSI_402BC000_BASE_ADDRESS
#define DT_VIDEO_MCUX_CSI_IRQ			DT_NXP_IMX_CSI_402BC000_IRQ_0
#define DT_VIDEO_MCUX_CSI_IRQ_PRI		DT_NXP_IMX_CSI_402BC000_IRQ_0_PRIORITY
#define DT_VIDEO_MCUX_CSI_NAME			DT_NXP_IMX_CSI_402BC000_LABEL
#define DT_VIDEO_MCUX_CSI_SENSOR_NAME		DT_NXP_IMX_CSI_402BC000_SENSOR_LABEL

/* End of SoC Level DTS fixup file */
