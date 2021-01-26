# 田間巡航艦
## 簡介
田間巡航艦是繼土砲一號後另一個農業機器人專案，主要用來實驗在田間巡航的技術可行性，以及用自己的屍體展示地雷在哪裡。

田間巡航艦與[土砲一號](https://github.com/aga3134/topower_v1)的架構/功能差別如下：
- 以市售遙控賽車車體為基礎，沒有機器手臂
- 使用24V鋰電池供電系統(5V電力系統已推不動這麼大台的車子)
- 用STM32控制板取代Arduino，整合感測器與馬達控制
- 用Jetson nano取代Raspberry pi，讓物體偵測可以直接在板端運行
- 從板端提供網頁介面，讓其他裝置可(從內網)透過網頁看車子狀態並控制車體運行
- 實作三種控制模式，除遙桿控制外，也測試了跟隨模式和自動巡航模式
- 提供網頁介面讓使用者設定巡航路線
- 整合gps和超音波測距做自動巡航
- 整合相機校正程序做apriltag 3d定位
- 整合realsense depth camera
- 整合yolov4,yolov5物體偵測
- 可從車體上傳影像到群眾標註，讓社群產生資料集供後續訓練

## 操作Demo
- 手動模式

    [![手動模式](https://img.youtube.com/vi/4c5Q6kc-0Go/0.jpg)](https://www.youtube.com/watch?v=4c5Q6kc-0Go)
- 跟隨模式

    [![跟隨模式](https://img.youtube.com/vi/PtjQ30sPv8I/0.jpg)](https://www.youtube.com/watch?v=PtjQ30sPv8I)
- 建立巡航路徑

    [![建立巡航路徑](https://img.youtube.com/vi/NoQLthXVnRM/0.jpg)](https://www.youtube.com/watch?v=NoQLthXVnRM)
- 自動模式 - 定點測試

    [![自動模式 - 定點測試](https://img.youtube.com/vi/zenvm4BipLA/0.jpg)](https://www.youtube.com/watch?v=zenvm4BipLA)
    - 本專案因軟硬體架構的運算能力/頻寬限制/加減速反應等因素，車體定位功能很容易迷失方向，因此無法實際在室外做自動巡航。(詳情請見下方的「已知問題」)
    
- YoloV4物體偵測(使用yolov4-tiny.weights)

    [![YoloV4物體偵測](https://img.youtube.com/vi/CkRs5hFfRVg/0.jpg)](https://www.youtube.com/watch?v=CkRs5hFfRVg)
- YoloV5物體偵測(使用yolov5s.pt)

    [![YoloV5物體偵測](https://img.youtube.com/vi/IxhhkE1drXM/0.jpg)](https://www.youtube.com/watch?v=IxhhkE1drXM)
- 物體偵測整合深度資訊

    [![物體偵測整合深度資訊](https://img.youtube.com/vi/M7vaHhkmBiU/0.jpg)](https://www.youtube.com/watch?v=M7vaHhkmBiU)
- 上傳群眾標註

    [![上傳群眾標註](https://img.youtube.com/vi/Cw8LqznywUo/0.jpg)](https://www.youtube.com/watch?v=Cw8LqznywUo)

## 相關文件
- [系統架構](https://docs.google.com/presentation/d/1bQ0sOPsP9fqFlR50wfA387S21WqCrcosq4zXLYWBXNc/edit?usp=sharing)
- [硬體元件](https://docs.google.com/spreadsheets/d/1nrL5iR8gq5ZfutR5axyTYxk2d--J_uWJTdhGib1Xtco/edit?usp=sharing)
- [3D列印件](https://cad.onshape.com/documents/602234696a3852f6f0ec04a2/w/bc81610e9b9f846b5bff4f5c/e/e0514e147666511ae9419ed6)
- [接線腳位](https://docs.google.com/spreadsheets/d/1xJIbeGXDL9BCHE3TLXravC8-m-VReMfphmmsiw1lIFk/edit?usp=sharing)
- [環境安裝與使用](https://docs.google.com/document/d/1BhkyFLj7_HSaj0HzsTvIo01meETMhIurQ3WTEAT3k7c/edit?usp=sharing)
- [已知問題](https://docs.google.com/document/d/1nQo5GTe0xTNbDD3QW71RGJT6YdIAVSU9dQT4iUrFV8Y/edit?usp=sharing)

## 資料夾結構
- nano
    - ros_ws/: ros workspace，放車體主程式
    - py3_ros_ws/: 使用python3的ros workspace，放YoloV4、YoloV5物體偵測程式
    - web/: 放控制車體的網頁程式
    - *.sh: 執行車體程式
    - experiment/: 開發時測試用的程式
- stm32/VapaaCruiser
    - VapaaCruiser.ioc: STM32Cube檔，設定控制板腳位
    - Src/: 控制板主程式
    - Inc/: 由STM32Cube產生的include檔
    - Drivers/: 由STM32Cube產生的STM32硬體函式庫
    - MDK-ARM/: 由STM32Cube產生的Keil uVision專案，在windows開發時可用
    - STM32CubeIDE/: 由STM32Cube產生的STM32CubeIDE專案，在ubuntu開發時可用
    - TrueSTUDIO/: 由STM32Cube產生的TrueSTUDIO專案，在ubuntu開發時可用
    - Makefile: 由STM32Cube產生的Makefile，在nano開發時可用

## 特別感謝
- 感謝Lock出借車體、控制板、電池、Realsense D435、Jetson nano
- 感謝哈爸提供韌體開發技術指導
- 感謝Victor提供STM32技術指導
- 感謝CH YoloV4、Yolov5技術文章與開源工具
