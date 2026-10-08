# Câmera no ESP32-P4: observações

Arquitetura: um único ESP32-P4 roda o Jiripoca V5 e a câmera.
OV2710 → ISP → PPA (recorte e redução) → JPEG → fila na PSRAM → mesmo cartão SD do log de voo.
Compila com o ESP-IDF 6.1 e o `esp_video` 2.5.0. **Ainda não foi testado em hardware.**

## Arquivos

| Arquivo | Função |
|---|---|
| `camera/camera.h` | Configurações (resolução, qualidade, filas) e formato do arquivo no SD |
| `camera/capture.c` | Liga a câmera, captura, recorta/reduz e codifica em JPEG (core 1, prioridade 3) |
| `camera/frameq.c` | Fila de frames na PSRAM com pré-disparo de ~2 s |
| `camera/recorder.c` | `task_camera`: grava os frames no SD do BOOST ao LANDED (core 1, prioridade 2) |
| `tools/cam_decode.py` | Converte `camN.mjp` em JPEGs, CSV com tempos e MP4 |

## Pinagem provisória

Os pinos em `main/header.h` estão mapeados na devkit ESP32-P4-EYE para testar em bancada sem conflitar com câmera, SD, USB e console.
Os marcados "J22" saem no header da devkit; os outros reaproveitam funções da devkit que não usamos (LCD, knob) e **não são acessíveis fisicamente**.
**Na placa final, só trocar os `#define`.** Pinos que a câmera e o SD usam e não podem ser reaproveitados (estão em `capture.c` e `header.h`):
I2C 13/14, XCLK 11, alimentação 12, reset 26, SD 39–44 e 46, Wi-Fi C6 9, USB 24/25, console 37/38.

## Antes de testar

- [ ] BMP390 e ICM20948 dividem o barramento I2C da câmera (pinos 10/12 do J22). Os endereços não colidem com o OV2710 nem com o QMA6100.
- [ ] Os pull-ups do I2C ficam no trilho 3V3_PERI, ligado por `board_power_init()` no início do `setup_peripherals`.
- [ ] Medir o consumo do P4 com a câmera e a queda de tensão no disparo pirotécnico.

## Primeiro teste em bancada

1. O log `capture` deve mostrar ~25 fps, o tamanho médio dos frames e a qualidade do JPEG.
2. Para gravar sem voo, trocar temporariamente em `recorder.c` a linha `bool record = status & BOOST;` por `bool record = true;`. **Não commitar essa troca.**
3. O log `CAMERA` mostra fps gravado, KB/s e frames descartados. Se `dropped` subir, o SD não acompanha: reduzir `CAM_JPEG_TARGET_BYTES`.
4. `python tools/cam_decode.py camN.mjp` (o MP4 precisa do ffmpeg).

## Decisões e limitações

- 640x480 é o recorte central de 1280x960 do 1080p (campo de visão ~1,5x mais fechado). Para campo total: `CAM_OUT` 480x270 e `CAM_CROP` 1920x1080. O PPA só aceita escalas múltiplas de 1/16.
- 25 fps é o máximo do OV2710.
- A câmera fica numa prioridade abaixo de todas as tasks de voo do core 1. As tasks internas do `esp_video` (ISP/IPA) têm prioridades próprias: medir com `vTaskGetRunTimeStats` em bancada.
- O SD roda em 4 bits a 40 MHz, dividido com o log de voo. O log para no LANDING, e o `task_sd` espera a câmera fechar o vídeo (até 60 s) antes de desmontar.
- Binário com 853 KB numa partição de app de 1 MB (17% livre). Se crescer, aumentar `app0` em `partition/16MB.csv`.
- Auto exposição e balanço de branco usam o JSON padrão do `esp_cam_sensor`. Se a imagem ficar ruim, testar o `ov2710_custom.json` do factory_demo.

## Mudanças no código do V5

- `header.h`: um único bloco de pinos para o P4 (o V5 tinha dois blocos com pinos repetidos); bits `CAM_EVT_*`.
- `system.c`: `board_power_init()` antes do I2C; sinaliza `CAM_EVT_BUS_READY`.
- `save.c`: SD no slot 0 do P4 com o LDO interno e o enable de alimentação; filas com timeout de 100 ms (corrige o bug em que `task_sd`/`task_lfs` travavam depois do LANDING/LANDED); espera a câmera antes de desmontar.
- `main.c`: cria a `task_camera`.
- `sdkconfig.defaults`: target `esp32p4`, PSRAM, OV2710 e ISP, mantendo as configurações do V5 (partição, FAT, log).

## Problemas do V5 ainda não corrigidos

- `acquire.c`: `vTaskDelay(500)` no disparo do drogue/main e até 900 ms nos bipes de armar travam a aquisição no momento crítico. O certo é desligar o pirotécnico por timer.
- `portENTER_CRITICAL` copia o `data_t` inteiro (com o estado do ESKF) com as interrupções do core desligadas.
- Filas alimentadas com timeout 0: perda de amostras sem contador.
- Nenhuma task sinaliza o `xInitEventGroup`: o `task_setup` cai em `setup_error` depois de 20 s, em silêncio.
- As tasks de sensores podem rodar antes de o `task_setup` criar o barramento I2C (corrida já existente no V5).
