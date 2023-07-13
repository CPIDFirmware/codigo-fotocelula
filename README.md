# codigo-luminaria

Código da Luminária com diferentes sensores.

# Descrição do código

Função criada para a fotocélula com sensores de temperatura, umidade, acelerômetro (para identificar se a fotocelula caiu, a assim um eventual poste), LDR (identificação se o ambiente está escuro para acionamento da luz automaticamente).

Todo o código foi pensado para comunicação LoRa e envio de pacote no modelo do Cayenne (já está estruturado e documentado na internet).
São utilizadas Tasks diferente para envio do pacote LoRa e acionamento das luzes, isto porquê o WHILE não permite delay.

No incio da TaskZero, que realiza envio de pacote LoRa, é desativdo o watchdog da task para que não necessite de delay.

Todos os tipos escolhidos pros sensores foram adaptados dos tipos de sensores já existentes no cayenne, de forma a não diminuir a precisão do dado e nem perder valores.

