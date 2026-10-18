# Câmera no ESP32-P4: observações

Arquitetura: um único ESP32-P4 roda o Jiripoca V5 e a câmera.
SC035HGS (640x480, 48 fps) → ISP (RAW10 → YUV420) → encoder H.264 por hardware → fila na PSRAM → mesmo cartão SD do log de voo.
Compila com o ESP-IDF 6.1, `esp_video` 2.5.0 e `esp_h264` 1.4.1. **Ainda não foi testado em hardware.**

## Arquivos

| Arquivo | Função |
|---|---|
| `camera/camera.h` | Configurações (resolução, fps, bitrate, GOP, filas) e formato do arquivo no SD |
| `camera/capture.c` | Liga a câmera e codifica cada frame em H.264 (core 1, prioridade 3) |
| `camera/frameq.c` | Fila de frames na PSRAM com pré-disparo, sempre começando num keyframe |
| `camera/recorder.c` | `task_camera`: grava os frames no SD do BOOST ao LANDED (core 1, prioridade 2) |
| `tools/cam_decode.py` | Converte `camN.vid` em `video.h264`, CSV com tempos e MP4 |

## Sensor SC035HGS

- **Hardware:** o conector da P4-EYE vem com o módulo OV2710. É preciso um módulo SC035HGS compatível com o FPC de 24 pinos e **MIPI de 1 lane**. Confirmar com o pessoal do hardware se o enable (GPIO12) e o reset (GPIO26) seguem a mesma ligação. O driver detecta o sensor sozinho pelo I2C.
- **Modo:** 640x480 RAW10 a 48 fps com clock de 20 MHz (`BOARD_CAM_XCLK_HZ` em `capture.c`). Existe um modo de 120 fps com 24 MHz; para usar, trocar o `sdkconfig.defaults`, o `BOARD_CAM_XCLK_HZ` e o `CAM_FPS`.
- **Imagem:** a resolução nativa é 640x480, então não há recorte e o campo de visão é o inteiro do sensor. O SC035HGS tem obturador global, que evita a distorção da imagem em movimento rápido (vibração, rolagem do foguete).

## H.264

- **Bitrate:** 2 Mbps (~250 KB/s no SD), controlado pelo próprio encoder (`CAM_H264_BITRATE`, `CAM_H264_QP_MIN/MAX`).
- **GOP:** um keyframe (IDR) por segundo (`CAM_H264_GOP`). Limita a perda: se um frame se corromper, o vídeo volta no próximo keyframe.
- **Frames P dependem do anterior.** Por isso:
  - com a fila cheia, o frame é descartado **antes** de codificar, e o vídeo continua válido (só fica um salto no tempo);
  - se um frame já codificado se perder, o código força um keyframe e descarta frames até ele;
  - o pré-disparo sempre começa num keyframe, então na prática guarda entre 1 e 2 s antes do BOOST.
- **Memória:** cada slot da fila tem 64 KB, para caber qualquer keyframe. A fila de 3 s a 48 fps usa ~9 MB de PSRAM. Se a placa final tiver menos PSRAM, reduzir `CAM_QUEUE_FRAMES` e `CAM_PRETRIGGER_FRAMES`.

## Pinagem provisória

Os pinos em `main/header.h` estão mapeados na devkit ESP32-P4-EYE para testar em bancada sem conflitar com câmera, SD, USB e console.
Os marcados "J22" saem no header da devkit; os outros reaproveitam funções da devkit que não usamos (LCD, knob) e **não são acessíveis fisicamente**.
**Na placa final, só trocar os `#define`.** Pinos usados pela câmera e pelo SD (em `capture.c` e `header.h`):
I2C 13/14, XCLK 11, alimentação 12, reset 26, SD 39–44 e 46, Wi-Fi C6 9, USB 24/25, console 37/38.

## Antes de testar

- [ ] BMP390 e ICM20948 dividem o barramento I2C da câmera (pinos 10/12 do J22). Conferir se o endereço I2C do SC035HGS não colide com 0x68 e 0x77.
- [ ] Os pull-ups do I2C ficam no trilho 3V3_PERI, ligado por `board_power_init()` no início do `setup_peripherals`.
- [ ] Medir o consumo do P4 com a câmera e a queda de tensão no disparo pirotécnico.

## Primeiro teste em bancada

1. O log `capture` deve mostrar ~48 fps, os kbit/s (perto de 2000) e os keyframes (~5 a cada 5 s).
2. Para gravar sem voo, trocar temporariamente em `recorder.c` a linha `bool record = status & BOOST;` por `bool record = true;`. **Não commitar essa troca.**
3. O log `CAMERA` mostra fps gravado, KB/s e frames descartados. Se `dropped` subir, o SD não acompanha: reduzir `CAM_H264_BITRATE`.
4. `python tools/cam_decode.py camN.vid`. O `video.h264` abre no VLC; o MP4 precisa do ffmpeg e é gerado sem recodificar.

## Decisões e limitações

- A câmera fica numa prioridade abaixo de todas as tasks de voo do core 1. As tasks internas do `esp_video` (ISP/IPA) têm prioridades próprias: medir com `vTaskGetRunTimeStats` em bancada.
- O SD roda em 4 bits a 40 MHz, dividido com o log de voo. O log para no LANDING, e o `task_sd` espera a câmera fechar o vídeo (até 60 s) antes de desmontar.
- O MP4 usa a taxa média de quadros. Se houver frames descartados, os tempos exatos de cada frame ficam no `index.csv`.
- Binário com 824 KB numa partição de app de 1 MB (20% livre). Se crescer, aumentar `app0` em `partition/16MB.csv`.
- Auto exposição e balanço de branco usam o JSON padrão do `esp_cam_sensor` para o SC035HGS.

## Mudanças no código do V5

- `header.h`: um único bloco de pinos para o P4 (o V5 tinha dois blocos com pinos repetidos); bits `CAM_EVT_*`.
- `system.c`: `board_power_init()` antes do I2C; sinaliza `CAM_EVT_BUS_READY`.
- `save.c`: SD no slot 0 do P4 com o LDO interno e o enable de alimentação; filas com timeout de 100 ms (corrige o bug em que `task_sd`/`task_lfs` travavam depois do LANDING/LANDED); espera a câmera antes de desmontar.
- `main.c`: cria a `task_camera`.
- `sdkconfig.defaults`: target `esp32p4`, PSRAM, SC035HGS e ISP, mantendo as configurações do V5 (partição, FAT, log).

## Problemas do V5 ainda não corrigidos

- `acquire.c`: `vTaskDelay(500)` no disparo do drogue/main e até 900 ms nos bipes de armar travam a aquisição no momento crítico. O certo é desligar o pirotécnico por timer.
- `portENTER_CRITICAL` copia o `data_t` inteiro (com o estado do ESKF) com as interrupções do core desligadas.
- Filas alimentadas com timeout 0: perda de amostras sem contador.
- Nenhuma task sinaliza o `xInitEventGroup`: o `task_setup` cai em `setup_error` depois de 20 s, em silêncio.
- As tasks de sensores podem rodar antes de o `task_setup` criar o barramento I2C (corrida já existente no V5).
