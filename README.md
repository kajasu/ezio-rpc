KC868-A6 ESP-IDF example
=========================

간단한 ESP-IDF 프로젝트 예제입니다. Kincony KC868-A6 보드 기반으로 시작하는 템플릿이며, LED blink와 로그 출력 예제를 포함합니다.

빌드 및 플래시(Windows, ESP-IDF PowerShell 환경):

```powershell
cd kc868_a6
idf.py set-target esp32
idf.py menuconfig   # (선택)
idf.py build
idf.py -p COM3 flash monitor
```

COM 포트와 타겟은 보드에 맞게 조정하세요.

파일 구조
- CMakeLists.txt - 프로젝트 진입
- main/main.c - Blink 예제
