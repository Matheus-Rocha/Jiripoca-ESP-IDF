# Câmera: observações da primeira versão

Sistema: OV2710 → ISP/PPA/JPEG (ESP32-P4-EYE, `camera_p4/`) → SPI → ESP32-S3 (`main/camera.c`) → SD.
Os dois firmwares compilam com o ESP-IDF 6.1. **Ainda não foram testados em hardware.**

## Antes de ligar os fios (hardware)

- [ ] Medir no J22 do P4-EYE quais dos pinos 1–4, 11 e 14 são 5 V e quais são GND.
- [ ] Confirmar que os sinais do J22 (GPIO50–54 do P4) estão em 3,3 V, o mesmo nível do S3.
- [ ] Confirmar na placa V5 que os GPIO 1, 2, 7, 11 e 36 do S3 estão livres. O GPIO36 não existe em módulos S3 com PSRAM octal.
- [ ] O P4 é alimentado pelo Jiripoca: medir o consumo (P4 + câmera + ISP) e a queda de tensão no disparo pirotécnico. As duas placas precisam de GND comum.

| Sinal | S3 | P4-EYE (J22) |
|---|---|---|
| SCLK | GPIO11 | pino 16 (GPIO52) |
| MOSI | GPIO7 | pino 15 (GPIO53) |
| MISO | GPIO2 | pino 17 (GPIO51) |
| CS | GPIO1 | pino 18 (GPIO50) |
| Frame pronto | GPIO36 | pino 13 (GPIO54) |

## Primeiro teste em bancada

1. Ligar só o P4: o log `capture` deve mostrar ~25 fps e o tamanho médio dos frames.
2. Ligar o S3: sem aviso `P4 not responding`, o link está funcionando em STANDBY.
3. Para gravar sem voo, trocar temporariamente em `main/camera.c` a linha `bool record = ...` por `bool record = !landed;`. **Não commitar essa troca.**
4. A linha `CAMERA` do S3 mostra fps, KB/s e erros a cada 5 s:
   - `crc` subindo: reduzir `CAM_SPI_FREQ_HZ` em `main/header.h`. Se estiver zerado, dá para tentar 20 MHz.
   - `P4 dropped` subindo: o link ou o SD não acompanham. Reduzir `CAM_JPEG_TARGET_BYTES` em `camera_p4/main/cam_p4.h` ou subir o SPI.
5. Converter o vídeo no computador: `python tools/cam_decode.py camN.mjp` (o MP4 precisa do ffmpeg).

## Decisões e limitações conhecidas

- 640x480 é um recorte central de 1280x960 do 1080p, então o campo de visão fica ~1,5x mais fechado. Para campo de visão total: `CAM_OUT` 480x270 e `CAM_CROP` 1920x1080 (escala 0,25). O PPA só aceita escalas múltiplas de 1/16.
- 25 fps é o máximo do OV2710 (nos dois modos, 1080p e 720p).
- Não há retransmissão: um pacote com CRC ruim descarta o frame inteiro (contadores `bad`/`crc`).
- Pré-disparo: o P4 guarda ~2 s antes do BOOST e até ~5 s de atraso do S3 (`CAM_PRETRIGGER_FRAMES` / `CAM_QUEUE_FRAMES`).
- LCD, módulo Wi-Fi (C6), lanterna e SD do P4 ficam desligados de propósito.
- Auto exposição e balanço de branco usam o JSON padrão do `esp_cam_sensor`. Se a imagem ficar ruim, testar o `ov2710_custom.json` do factory_demo via `CONFIG_CAMERA_OV2710_CUSTOMIZED_IPA_JSON_CONFIGURATION_FILE`.
- Se o `fopen` do log de voo falhar no boot, o `task_sd` espera até 60 s pela câmera e depois desmonta o SD. Caso degenerado, a câmera perde o cartão.
- O FAT está sem nomes longos (`CONFIG_FATFS_LFN_NONE`): `cam%lu.mjp` suporta contador até 99999.

## Mudanças no código existente (avisar o João)

- `save.c`: as filas do `task_sd` e do `task_lfs` agora têm timeout de 100 ms. **Corrige um bug da V5:** depois do LANDING/LANDED as filas param de receber dados, as tasks travavam em `portMAX_DELAY`, os arquivos de voo nunca eram fechados e o contador do NVS nunca era atualizado.
- `save.c`: o SD avisa a câmera quando monta ou falha, e espera a câmera fechar o vídeo (`CAM_EVT_DONE`, até 60 s) antes de desmontar. A câmera grava até o LANDED, e o log de voo continua parando no LANDING.
- `header.h`: pinos da câmera e bits `CAM_EVT_*`. `main.c`: criação da task (core 1, prioridade 2, a menor entre as tasks de voo).

## Problemas da V5 que não são da câmera (não corrigidos)

- `header.h` define pinos em dobro: I2C e SPI2 nos GPIO 8/9, `SS` e `SPI2_MOSI` no 10, `GPS_RX` e `LORA_DIO1` no 14, `LED_GPIO` e `MAIN_DEPLOY` no 15, `BUZZER_GPIO` redefinido no 21 (que também é `GPS_TX`), `HX_DOUT` no 38 (o antigo buzzer), `SD_CMD` e `SD_PWR` no 46.
- `SD_CLK` no GPIO43 é o TX do console UART0 (`CONFIG_ESP_CONSOLE_UART_DEFAULT`).
- `LORA_CS` no GPIO20 é o D+ do USB, então o USB-JTAG deixa de funcionar.
- GPIO45 e GPIO46 são pinos de strapping do S3.
- Nenhuma task sinaliza o `xInitEventGroup`: o `task_setup` sempre cai em `setup_error` depois de 20 s, em silêncio.
- `sd_header`/`lfs_header` são criados e nunca gravados nos arquivos.

## sdkconfig

O ESP-IDF 6.1 regenerou o `sdkconfig` do S3, e ele **não** foi commitado. Só commitar se a equipe padronizar no 6.1.
