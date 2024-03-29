/*
**
** Copyright 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef __FLASH_TUNING_CUSTOM_H__
#define __FLASH_TUNING_CUSTOM_H__



int getDefaultStrobeNVRam(int sensorType, void* data, int* ret_size);

FLASH_PROJECT_PARA& cust_getFlashProjectPara(int AEMode, NVRAM_CAMERA_STROBE_STRUCT* nvrame);
int cust_isNeedAFLamp(int flashMode, int afLampMode, int isBvHigherTriger);


int cust_getFlashModeStyle(int sensorType, int flashMode);
int cust_getVideoFlashModeStyle(int sensorType, int flashMode);
void cust_getEvCompPara(int& maxEvTar10Bit, int& indNum, float*& evIndTab, float*& evTab, float*& evLevel);

int cust_isSubFlashSupport();

#endif //#ifndef __FLASH_TUNING_CUSTOM_H__

