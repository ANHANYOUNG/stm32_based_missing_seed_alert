# 파종기 결주 알림 시스템

파종 작업 중 씨앗 미배출(결주)을 감지하고 HMI 디스플레이에 상태를 표출하는 시스템.

## 프로젝트 구조
- `Altium/` : 하드웨어 회로도 및 PCB 설계 파일
- `MCU/` : STM32 펌웨어 소스코드 (STM32CubeIDE)
- `Nextion HMI/` : 작업자 확인용 디스플레이 UI 파일 (.HMI)

## 하드웨어 및 개발 환경
- MCU : STM32F411RET6
- Firmware : STM32CubeIDE, HAL API
- HW Design : Altium Designer
- Display : Nextion HMI

## 빌드 및 실행
1. `Altium/` 내 설계 파일 기반으로 보드 제작.
2. Nextion Editor를 사용해 `prox_en_v2.HMI` 파일을 디스플레이에 다운로드.
3. STM32CubeIDE에서 `MCU/f411re/` 프로젝트 임포트 후 빌드 및 바이너리 다운로드.
