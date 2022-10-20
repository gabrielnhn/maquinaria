# maquinaria

Conectando no maquinaria:

1. Conecte o RaspberryPi à internet (de seu roteador do celular)
    Se nunca foi conectando anteriormente, será necessário ligá-lo no monitor e usar a GUI (Interface gráfica) para conectá-lo - Clicar com o mouse no botão de conexões à rede como em um computador comum.

2. Descubra o endereço IP do RaspberryPi
    #### Se não está conectado no monitor:
    1. Conecte outro dispositivo à rede do celular.
    2. Use o comando ```ifconfig``` para descobrir o endereço deste dispositivo.
        Se o seu endereço é AAA.BBB.CCC.DDD, temos que o endereço da rede é AAA.BBB.CCC.000
    
    3. Use o comando `nmap <Rede>/24` para identificar os hosts desta rede, buscando pelo host que tem portas ssh abertas.
    
    #### Se ligado no monitor, um ```ifconfig``` já é o suficiente.

3. Conecte-se por ssh.
    Use o comando ```ssh pi@<Endereço>```. A senha é "naomexar".
    
4. Na pasta `~/follower` estão os programas de interesse; em específico, `program.py` e `DC_Motor_pi.py`.

5. Use o comando ```$ python3 program.py``` na pasta para iniciar o programa. ~~Digite start para que o seguidor comece a andar~~.