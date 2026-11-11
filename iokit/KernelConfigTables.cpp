/*
 * Copyright (c) 1998-2012 Apple Inc. All rights reserved.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_START@
 *
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. The rights granted to you under the License
 * may not be used to create, or enable the creation or redistribution of,
 * unlawful or unlicensed copies of an Apple operating system, or to
 * circumvent, violate, or enable the circumvention or violation of, any
 * terms of an Apple operating system software license agreement.
 *
 * Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this file.
 *
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_END@
 */
/*
 * NOTICE: This file was modified by SPARTA, Inc. in 2005 to introduce
 * support for mandatory and extensible security protections.  This notice
 * is included in support of clause 2.2 (b) of the Apple Public License,
 * Version 2.0.
 */

const char * gIOKernelConfigTables =
    "("
#if defined(ARM64_BOARD_CONFIG_MT6781)
    "   {"
    "     'IOClass'         = MapplePlatformExpert;"
    "     'IOProviderClass' = IOPlatformExpertDevice;"
    "     'IONameMatch'     = 'mapple,platform';"
    "     'IOProbeScore'    = 10000:32;"
    "   },"
    "   {"
    "     'IOClass'         = MappleCPU;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'qcom,kryo470';"
    "     'IOProbeScore'    = 10000:32;"
    "   },"
    "   {"
    "     'IOClass'         = AppleKeyStore;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = AppleKeyStore;"
    "   },"
    "   {"
    "     'IOClass'         = IOWatchdog;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = IOWatchdog;"
    "   },"
    "   {"
    "     'IOClass'         = AppleMobileFileIntegrity;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = AppleMobileFileIntegrity;"
    "   },"
    "   {"
    "     'IOClass'         = IOCoreSurfaceRoot;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = IOCoreSurfaceRoot;"
    "   },"
    "   {"
    "     'IOClass'         = AppleCLCD;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mapple,display';"
    "   },"
    "   {"
    "     'IOClass'         = MappleGIC;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'arm,gic-v3';"
    "   },"
    "   {"
    "     'IOClass'         = MTKKeypad;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-keypad';"
    "   },"
    "   {"
    "     'IOClass'         = MTKPwrap;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-pwrap';"
    "   },"
    "   {"
    "     'IOClass'         = MTKEINT;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-eint';"
    "   },"
    "   {"
    "     'IOClass'         = MT6358PMIC;"
    "     'IOProviderClass' = MappleDTNub;"
    "     'IONameMatch'     = 'mediatek,mt6366-pmic';"
    "   },"
    "   {"
    "     'IOClass'         = MT6358Keys;"
    "     'IOProviderClass' = MappleDTNub;"
    "     'IONameMatch'     = 'mediatek,mt6366-keys';"
    "   },"
    "   {"
    "     'IOClass'         = MTKGPIO;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-gpio';"
    "   },"
    "   {"
    "     'IOClass'         = MTKI2C;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-i2c';"
    "   },"
    "   {"
    "     'IOClass'         = GT9896;"
    "     'IOProviderClass' = MappleDTNub;"
    "     'IONameMatch'     = 'goodix,gt9896';"
    "   },"
    "   {"
    "     'IOClass'         = BQ27Z561Battery;"
    "     'IOProviderClass' = MappleDTNub;"
    "     'IONameMatch'     = 'ti,bq27z561';"
    "   },"
    "   {"
    "     'IOClass'         = MT6631WLAN;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-wlan';"
    "   },"
    "   {"
    "     'IOClass'         = IONetworkStack;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = IONetworkStack;"
    "   },"
    "   {"
    "     'IOClass'         = MappleGPUAccelerator;"
    "     'IOProviderClass' = IOPlatformDevice;"
    "     'IONameMatch'     = 'mediatek,mt6781-mali';"
    "   },"
    "   {"
    "     'IOClass'         = AppleM2ScalerCSCDriver;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = AppleM2ScalerCSCDriver;"
    "   },"
    "   {"
    "     'IOClass'         = MTKJPEGEncoder;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = MTKJPEGEncoder;"
    "   },"
    "   {"
    "     'IOClass'         = MappleHIDButtons;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = MappleHIDButtons;"
    "   },"
    "   {"
    "     'IOClass'         = MappleHIDTouch;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = MappleHIDTouch;"
    "   },"
    "   {"
    "     'IOClass'         = MapplePowerSource;"
    "     'IOProviderClass' = IOResources;"
    "     'IOMatchCategory' = MapplePowerSource;"
    "   },"
#endif
    "   {"
    "     'IOClass'         = IOPanicPlatform;"
    "     'IOProviderClass' = IOPlatformExpertDevice;"
    "     'IOProbeScore'    = 0:32;"
    "   }"
    ")";
