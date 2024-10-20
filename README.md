# CHECKPOINT 5

## Repositório dedicado ao Checkpoint 5da matéria de Edge Computing & Computer Systems

Para acessar os Dashboards com a minha última leitura acesse: http://18.231.116.149:1880/ui/

Caso os dados não estejam atualizando, é porquê eu não estou usando o sistema no momento

Envie um email para leonardofarias.comercial@gmail.com que eu poderei fazer uma demonstração do sistema operando

## Integrantes:

Leonardo de Farias - RM: 555211

Giancarlo Cestarolli - RM: 555248

Gustavo Laur - RM: 556603

## Preparando ambiente
- Instalar o Arduino IDE 2
- Vá para File → Preferences e copie e cole os seguintes links no espaço Additional Boards Manager URLs:
http://arduino.esp8266.com/stable/package_esp8266com_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- No menu lateral, clique no Board Manager e baixe o "ESP32 By Espressif Systems"
- Ainda no menu lateral, clique no Library Manager, Pesquise e baixe as seguintes bibliotecas:
PubSubClient by Nick O'Leary
SparkFun MAX3010x Pulse and Proximity Sensor Libray by SparkFun Electronics
- Não esqueça de substituir as credenciais de WiFi para utilizar o código
- Caso queira utilizar o código em um projeto próprio, será necessário substituir as outras credenciais do mqtt nesta mesma parte(Entre as linhas 7 e 13)

## Materiais:
- Sensor MAX30102
- Placa ESP32
- Jumpers (cabos de conexão)
- LED
- Resistor
- Protoboard (opcional, para facilitar a montagem)

## Conexões 
- MAX30102 VIN → 3.3V do ESP32
- MAX30102 GND → GND do ESP32
- MAX30102 SDA → Pino 21 (SDA) do ESP32
- MAX30102 SCL → Pino 22 (SCL) do ESP32
- LED + → Pino 25 do ESP32
- LED - → GND dp ESP32
