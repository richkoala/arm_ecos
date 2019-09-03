#include "xbram_control.h"

XBram Bram;

int XBram_CfgInitialize(XBram *InstancePtr,
			XBram_Config *Config,
			UINTPTR EffectiveAddr)
{
	/*
	 * Assert arguments
	 */
	Xil_AssertNonvoid(InstancePtr != NULL);

	/*
	 * Set some default values.
	 */
	InstancePtr->Config.CtrlBaseAddress = EffectiveAddr;
	InstancePtr->Config.MemBaseAddress = Config->MemBaseAddress;
	InstancePtr->Config.MemHighAddress = Config->MemHighAddress;
	InstancePtr->Config.DataWidth = Config->DataWidth;
	InstancePtr->Config.EccPresent = Config->EccPresent;
	InstancePtr->Config.FaultInjectionPresent =
					Config->FaultInjectionPresent;
	InstancePtr->Config.CorrectableFailingRegisters =
					Config->CorrectableFailingRegisters;
	InstancePtr->Config.CorrectableFailingDataRegs =
					Config->CorrectableFailingDataRegs;
	InstancePtr->Config.UncorrectableFailingRegisters =
					Config->UncorrectableFailingRegisters;
	InstancePtr->Config.UncorrectableFailingDataRegs =
					Config->UncorrectableFailingDataRegs;
	InstancePtr->Config.EccStatusInterruptPresent =
					Config->EccStatusInterruptPresent;
	InstancePtr->Config.CorrectableCounterBits =
					Config->CorrectableCounterBits;
	InstancePtr->Config.WriteAccess = Config->WriteAccess;

	/*
	 * Indicate the instance is now ready to use, initialized without error
	 */
	InstancePtr->IsReady = XIL_COMPONENT_IS_READY;
	return (XST_SUCCESS);
}


void InitializeECC(XBram_Config *ConfigPtr, u32 EffectiveAddr)
{
	u32 Addr;
	volatile u32 Data;

	if (ConfigPtr->EccPresent &&
	    ConfigPtr->EccOnOffRegister &&
	    ConfigPtr->EccOnOffResetValue == 0 &&
	    ConfigPtr->WriteAccess != 0) {
		for (Addr = ConfigPtr->MemBaseAddress;
		     Addr < ConfigPtr->MemHighAddress; Addr+=4) {
			Data = XBram_In32(Addr);
			XBram_Out32(Addr, Data);
		}
		XBram_WriteReg(EffectiveAddr, XBRAM_ECC_ON_OFF_OFFSET, 1);
	}
}


int BramControl_init(u16 DeviceId)
{
	int Status;
	XBram_Config *ConfigPtr;

	/*
	 * Initialize the BRAM driver. If an error occurs then exit
	 */

	/*
	 * Lookup configuration data in the device configuration table.
	 * Use this configuration info down below when initializing this
	 * driver.
	 */
	ConfigPtr = XBram_LookupConfig(DeviceId);
	if (ConfigPtr == (XBram_Config *) NULL) {
		return XST_FAILURE;
	}

	Status = XBram_CfgInitialize(&Bram, ConfigPtr,
				     ConfigPtr->CtrlBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    InitializeECC(ConfigPtr, ConfigPtr->CtrlBaseAddress);

	/*
	 * Execute the BRAM driver selftest.
	 */
	//Status = XBram_SelfTest(&Bram, 0);
	//if (Status != XST_SUCCESS) {
	//	return XST_FAILURE;
	//}

	return XST_SUCCESS;
}


