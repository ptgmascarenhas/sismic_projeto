// LCD_I2C - Acessar LCD usando porta I2C
#include <msp430.h>

#define BR100K      10      //100 kbps
#define BR50K       20      // 50 kbps
#define BR10K       100     // 10 kbps

#define PCF 0x27    //Endereço PCF8574T

// Bits para controle do LCD
#define BIT_RS   BIT0
#define BIT_RW   BIT1
#define BIT_E    BIT2
#define BIT_BL   BIT3
#define BIT_D4   BIT4
#define BIT_D5   BIT5
#define BIT_D6   BIT6
#define BIT_D7   BIT7

// Tempos do DHT22
#define Tbe     10490   //aproximadamente 1 ms
#define Tgo     320     //aproximadamente 300 us
#define Trel    84      //aproximadamente 80 us
#define Treh    84      //aproximadamente 80 us
#define Tlow    53      //aproximadamente 50 us
#define Th0     28      //aproximadamente 26 us
#define Th1     74      //aproximadamente 70 us
#define Ten     60      //mais que 50 us

/////////////////////////////////////////////

void pcf_write(char dado); // envia para o escravo

void lcd_write_nib(char);       // escreve nibble (0000 xxxx)
void lcd_write_byte(char, int); // escreve byte
void lcd_char(char);            // imprime dado
void lcd_num(int, char);        // imprime numero
void lcd_str(char*);            //imprime no LCD a string apontada por pt até encontrar o '\0'

void lcd_rs_rw(int, int);   // RS e RW
void lcd_blink_enable(int); // define o enable
void lcd_BL(int);           // define o BL

void lcd_clr(void);  // apaga todo o display e posiciona o cursor na primeira linha da primeira coluna (instrução)
void lcd_go_to(int); // posiciona o cursor onde o usuario quiser
void lcd_inic(void); // inicializa o lcd

void config_pinos(void); // configura os pinos
void config_I2C(void);   // configura as comunicacao
void delay(long limite); // atrasa

int humidityInt(void);
int temperatureInt(void);
int checksumInt(void);
void dht22(void);
void dht22_data(void);
void dht22_result(void);

void aviso(void);

void hc5_config_serial(void);
void hc5_config_pinos(void);
void hc5_write_temp(void);

/////////////////////////////////////////////

int porta = 0;       //valor que sera enviado
int position = 0x00; //posicao do cursor

int tBaixo, tAlto, dado[40], i, hh[8], hl[8], th[8], tl[8], cs[8], t, h, checksum, sum;
int aux1, aux2, aux3;
int tk, tc, td, tu; //variaveis para a funcao de numero no lcd
int hk, hc, hd, hu; //variaveis para a funcao de numero no lcd


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    __delay_cycles(30000);

    config_pinos();
    config_I2C();
    lcd_inic();

    hc5_config_serial();
    hc5_config_pinos();
    UCA0CTL1 &= ~UCSWRST;

    //lcd_write_byte(0xF, 0); //ativa o mostrador, com cursor
    lcd_write_byte(0xC, 0); //ativa o mostrador, sem cursor


        //coloca o p2.0 como saida em 1
        P2DIR |= BIT0;  //2.0 como saida
        P2OUT |= BIT0;  //2.0 em 1
        __delay_cycles(1000);

        dht22();

        __delay_cycles(1000);

        dht22_data();

        h = humidityInt();
        t = temperatureInt();

        dht22_result();

        if(t < 100 || t > 400 || h < 300 || h > 800)
            aviso();


    while(1){
        while(!(UCA0IFG & UCRXIFG)); // Espera receber comando do botão configurado no App
        if(UCA0RXBUF == 0x99){ // Se o botão tiver configurado para receber o código HEX 0x99, entra nesse if
            hc5_write_temp(); // Faz o print da temperatura no console do app
            __delay_cycles(200); // Delay para estabilizar
        }
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////


/**
 * Função que faz a configuração dos pinos que conectam o hc05 e o msp430 fisicamente
 * Para configurar RX e TX sem ser por pino, deve-se colocar o TX do outro dispositivo no RX do HC05 e o TX do HC05 no RX do outro dispositivo.
 * */
void hc5_config_pinos(void){
    P3DIR |= BIT3; // P3.3 : RX do HC05
    P3DIR &= ~BIT4; // P3.4 : TX do HC05
    P3REN |= BIT4; // Resistor Enable do P3.4
    P3OUT |= BIT4; // PullUp do P3.4
    P3SEL |= BIT3; // Seleciona pino P3.3
    P3SEL |= BIT4; // Seleciona pino P3.4
}

/**
 * Função que faz a configuração da comunicação serial entre o hc05 e o msp430
 * */
void hc5_config_serial(void){
    UCA0CTL1 = UCSWRST;
    UCA0CTL1 = UCSSEL_2 | UCSWRST;

    UCA0BRW = 6;
    UCA0MCTL = UCBRF_13 | UCBRS_0 | UCOS16;
}

/**
 * Função que faz a leitura da temperatura quando selecionado o botão configurado no app Bluetooth Terminal HC-05
 * A escrita da temperatura para o app é feito transmitindo cada caracter a cada transmissção (UCA0TXBUF)
 * Pra configurar o botão no app, baixe o app Bluetooth Terminal HC-05, parear com o HC-05, selecione um dos botões na parte de baixo, Digite nome, selecione HEX e informe o command que será enviado pro App quando clicar no botão
 * */
void hc5_write_temp(void){
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x0A; // \n : new lina, pula uma linha
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x54; // T
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x3A; // :
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = tc+0x30; // 2
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = td+0x30; // 2
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x2E; // .
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = tu+0x30; // 5
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 'º'; // º
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x43; //C
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x0A; // \n
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x55; // U
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x3A; // :
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = hc+0x30; // 6
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = hd+0x30; // 5
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x2E; // .
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = hu+0x30; // 9
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x25; // %
    __delay_cycles(200);

    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x0A; // \n
    __delay_cycles(200);
}

int p1in, p1out, p1ren, p1sel, p1dir;

void analisap1(void){
    p1in = P1IN;
    p1out = P1OUT;
    p1ren = P1REN;
    p1sel = P1SEL;
    p1dir = P1DIR;
}

void dht22(void){
    P1IN = 0;
    P1OUT = 0;
    P1REN = 0;
    P1SEL = 0;
    P1DIR = 0;

    analisap1();

    __delay_cycles(3000000);

    TA0CTL = TASSEL_2|MC_1;     //SMCLK (1MHz), UP

    //faz pino como saida e envia 1 na saida
    P1DIR |= BIT2;  //1.2 como saida
    P1OUT |= BIT2;  //1.2 em 1
    P1SEL &= ~BIT2; //Tira a funcao de captura

    analisap1();

    __delay_cycles(3000000);

    //envia 0 pra saida e espera ccro = 10490
    TA0CCR0 = Tbe;
    TA0CTL |= TACLR;                //zera o timer
    P1OUT &= ~BIT2;                 //envia 0
    while( (TA0CTL & TAIFG) == 0 ); //espera dar 1ms
    TA0CTL &= ~TAIFG;

    //coloca o pino como entrada
    P1DIR &= ~BIT2; //1.2 como entrada
    P1SEL |= BIT2;  //Da a funcao de captura de tempo
    P1REN |= BIT2;  //1.2 com resist
    P1OUT |= BIT2;  //1.2 com pull up


    //espera a resposta do dht
    while( (P1IN & BIT2) == BIT2); //fica preso em 1 (Tgo)


    TA0CCTL1 = CM_1|CCIS_0|SCS|CAP; //captura flanco de subida, sincrona
    TA0CTL |= TACLR;
    while( (P1IN & BIT2) == 0 );   //fica recebendo o Trel
    //tBaixo = TA0R;
    TA0CCTL1 &= ~CCIFG;

    TA0CCTL1 = CM_2|CCIS_0|SCS|CAP; //captura flanco de descida, sincrona
    TA0CTL |= TACLR;
    while( (P1IN & BIT2) == BIT2 );   //fica recebendo o Treh
    //tAlto = TA0R;
    TA0CCTL1 &= ~CCIFG;

    TA0CCTL1 = CM_2|CCIS_0|SCS|CAP; //captura flanco de descida, sincrona

    for(i = 0; i < 40; i++){

       while( (P1IN & BIT2) == 0 ); //espero enquanto for 0 (Tlow)
       TA0CTL |= TACLR;            //foi pra 1 Th

       while( (P1IN & BIT2) == BIT2 );   //fica recebendo o Th

       if(TA0CCR1 < 45) //dado recebido vale 0
           dado[i] = 0;
       else             //dado recebido vale 0
           dado[i] = 1;

       TA0CCTL1 &= ~CCIFG;
    }

    TA0CCTL1 = CM_1|CCIS_0|SCS|CAP; //captura flanco de subida, sincrona
    TA0CTL |= TACLR;
    while( (P1IN & BIT2) == 0 );   //fica recebendo o Ten
    //tBaixo = TA0R;
    TA0CCTL1 &= ~CCIFG;


    analisap1();


    //faz pino como saida e envia 1 na saida
    P1DIR |= BIT2;  //1.2 como saida
    P1OUT |= BIT2;  //1.2 em 1
    P1SEL &= ~BIT2; //Tira a funcao de captura
    P1REN &= ~BIT2;  //1.2 sem resist

    analisap1();

}


void dht22_data(void){
    for(i = 0; i < 8; i++){
        hh[i] = dado[i];
        hl[i] = dado[i+8];
        th[i] = dado[i+16];
        tl[i] = dado[i+24];
        cs[i]  = dado[i+32];
    }
}

void dht22_result(void){
    if(checksumInt()){
        lcd_str("T: ");
        lcd_num(t, 't');
        lcd_char(' ');
        lcd_char(223);
        lcd_char('C');

        lcd_go_to(0x40);
        lcd_str("U: ");
        lcd_num(h, 'h');
        lcd_str(" %");
    }
    else{
        lcd_str("Erro1: falha na comunicacao");
    }
}

int humidityInt(void){
    int humidity;

    humidity = 1*hl[7]    + 2*hl[6]    + 4*hl[5]     + 8*hl[4]
             + 16*hl[3]   + 32*hl[2]   + 64*hl[1]    + 128*hl[0]

             + 256*hh[7]  + 512*hh[6]  + 1024*hh[5]  + 2048*hh[4]
             + 4096*hh[3] + 8192*hh[2] + 16384*hh[1] + 32768*hh[0];

    return humidity;
}

int temperatureInt(void){
    int temperature;

    temperature =  1*tl[7]    + 2*tl[6]    + 4*tl[5]     + 8*tl[4]
                 + 16*tl[3]   + 32*tl[2]   + 64*tl[1]    + 128*tl[0]

                 + 256*th[7]  + 512*th[6]  + 1024*th[5]  + 2048*th[4]
                 + 4096*th[3] + 8192*th[2] + 16384*th[1];

    temperature = th[0] ? -temperature : temperature;

    return temperature;
}

int checksumInt(void){
    checksum =  1*cs[7] + 2*cs[6]  + 4*cs[5]  + 8*cs[4]
             + 16*cs[3] + 32*cs[2] + 64*cs[1] + 128*cs[0];

    sum = (h + t) & 0xFF; //para ficar somente o lsb

    i = (h+t);

    if( (checksum - 2) == sum ||(checksum - 1) == sum)
        return 1;
    else
        return 0;
}

void aviso(void){
    P6DIR |= BIT5;
    P6OUT |= BIT5;
}


////////////////////////////////////////////////////////////////////////////////////

// Escrever dado na porta
void pcf_write(char dado){
    UCB0I2CSA = PCF;                          //Endereço do Escravo

    UCB0CTL1 |= UCTR    |                     //Mestre transmissor
                UCTXSTT;                      //Gerar START e envia endereço
    while ( (UCB0IFG & UCTXIFG) == 0);        //Esperar TXIFG (completar transm.)

    if ( (UCB0IFG & UCNACKIFG) == UCNACKIFG){ //NACK?
        P1OUT |= BIT0;                        //Acender LED Vermelho
        while(1);                             //Se NACK, prender
    }

    UCB0TXBUF = dado;                         //Dado a ser escrito se começa a transmitir enao recebeu nack
    while ( (UCB0IFG & UCTXIFG) == 0);        //Esperar Transmitir

    UCB0CTL1 |= UCTXSTP;                      //Gerar STOP
    while ( (UCB0CTL1 & UCTXSTP) == UCTXSTP); //Esperar STOP
    delay(50);                                //Atraso p/ escravo perceber stop
}

//Ler dado da porta
int pcf_read(void){
    int dado = 0;
    UCB0CTL1 &= ~UCTR;   //mestre receptor
    UCB0CTL1 |= UCTXSTT; //gerar start

    while((UCB0CTL1 & UCTXSTT) == UCTXSTT); //preso enquanto estiver start
    UCB0CTL1 |= UCTXSTP;                    //se saiu, manda stop

    while((UCB0IFG & UCRXIFG) == 0);        //preso enquanto nao receber nada
    dado = UCB0RXBUF;                       //recebeu, armaze em dado

    delay(10000);

    return dado;
}

////////////////////////////////////////////////////////////////////////////////////

void lcd_write_nib(char nib){
    porta &= 0x0F;       //zera os bits de dado
    porta |= nib;        //coloca novos bits de dado
    lcd_BL(1);           //liga o backlight
    pcf_write(porta);    //envia pro pcf
    lcd_blink_enable(1); //pulsa o enable
    pcf_write(porta);    //envia novamente com enable em 1
    __delay_cycles(50);
    lcd_blink_enable(0); //desliga o enable
    pcf_write(porta);    //envia com o enable desligado
}

void lcd_write_byte(char byte, int rs){
    char nibF0 = byte & 0xf0; // separa o MSN
    char nib0F = byte & 0x0f; // separa o LSN
    nib0F = nib0F << 4;       // roda o LSN 4 posicoes

    if(rs == 0) nibF0 &= ~BIT_RS; // RS = 0 -> instrucao

    lcd_write_nib(nibF0);
    lcd_write_nib(nib0F);
    //OBS: nao precisa mandar o RS=0 de novo pois a funcao conserva o LSN
}

void lcd_char(char dado){   //enviar a posicao em hex 0x00-0f, 0x40-4f
    if(position == 0x10){   //se for maior que a linha 1 deve ir pra linha 2
        lcd_go_to(0x40);    //indo para a primeira posicao da segunda linha
    }
    if(position == 0x50){   //se for maior que a linha 2 deve ir para o inicio
        lcd_go_to(0x00);    //indo para a primeira posicao da primeira linha
    }

    lcd_rs_rw(1,0);          //como e um dado envia o RS = 1
    lcd_write_byte(dado, 1); //envia o dado e fala que nao é instrucao
    position++;

    __delay_cycles(1000);    //espera para enviar o proximo char
}

void lcd_num(int num, char c){        //converte de 0 a 1000
    int flag = 0;

    if(c == 't'){
        if(num > 999){
            tk = (num / 1000);
            num = num - tk*1000;
            lcd_char(tk+0x30);
            flag = 1;
        }
        if(num > 99 || flag == 1){
            tc = (num / 100);
            num = num - tc*100;
            lcd_char(tc+0x30);
            flag = 1;
        }
        if(num > 9 || flag == 1){
            td = (num / 10);
            num = num - td*10;
            lcd_char(td+0x30);
        }

        lcd_char('.');

        tu = (num);
        lcd_char(tu+0x30);
    }

    if(c == 'h'){
        if(num > 999){
            hk = (num / 1000);
            num = num - hk*1000;
            lcd_char(hk+0x30);
            flag = 1;
        }
        if(num > 99 || flag == 1){
            hc = (num / 100);
            num = num - hc*100;
            lcd_char(hc+0x30);
            flag = 1;
        }
        if(num > 9 || flag == 1){
            hd = (num / 10);
            num = num - hd*10;
            lcd_char(hd+0x30);
        }

        lcd_char('.');

        hu = (num);
        lcd_char(hu+0x30);
    }

}


void lcd_str(char *message){
    while(*message != '\0'){
        char aux = *message;
        lcd_char(aux);
        ++message;
    }
}

////////////////////////////////////////////////////////////////////////////////////

void lcd_BL(int on){ //se for 1 ele liga o BL
    porta = on ? (porta |= BIT_BL) : (porta &= BIT_BL);
}
void lcd_rs_rw(int rs, int rw){
    porta = rs ? (porta | BIT_RS) : (porta & ~BIT_RS);
    porta = rw ? (porta | BIT_RW) : (porta & ~BIT_RW);
}
void lcd_blink_enable(int enable){ //se for 1 ele liga o enable
    porta = enable ? (porta | BIT_E) : (porta & ~BIT_E);
}

////////////////////////////////////////////////////////////////////////////////////

void lcd_clr(void){
    lcd_rs_rw(0, 0);
    lcd_write_byte(0x01, 0); //instrucao de limpar o lcd
    position = 0x00;

    __delay_cycles(90000);   //para dar o tempo do clear
}

void lcd_go_to(int local){
    lcd_rs_rw(0, 0);
    lcd_write_byte((local|0x80), 0); //instrucao de ir para um lugar (0x80+p)

    __delay_cycles(100);

    if(local <= 0x0F){
        position = local;
    }else {
        if(local >= 0x40 && local <= 0x4f)
            position = local;
        else
            position = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////

//inicialização de 4 bits
void lcd_inic(void){
    __delay_cycles(300000);        //esperar mais de 15 ms
    lcd_write_nib(BIT5 | BIT4);    //0011

    __delay_cycles(90000);         //esperar mais de 4,1 ms
    lcd_write_nib(BIT5 | BIT4);    //0011

    __delay_cycles(20000);         //esperar mais 100 micro s
    lcd_write_nib(BIT5 | BIT4);    //0011

    __delay_cycles(1000);
    lcd_write_nib(BIT5);           //0010

    __delay_cycles(100);
    lcd_write_byte((BIT5|BIT3),0); //0010 1000

    __delay_cycles(100);
    lcd_write_byte(BIT3, 0);       //0000 1000 MOSTRADOR DESATIVADO

    __delay_cycles(100);
    lcd_write_byte(BIT0, 0);       //0000 0001 LIMPAR MOSTRADOR

    __delay_cycles(100);
    lcd_write_byte((BIT2|BIT1),0); //0000 0011 ENTRADA E SAIDA
}

////////////////////////////////////////////////////////////////////////////////////

// Configurar Pinos I2C - UCSB0
// P3.0 --> SDA
// P3.1 --> SCL
void config_I2C(void){
    P3SEL |=  BIT0;      // Usar módulo dedicado
    P3REN |=  BIT0;      // Habilitar resistor
    P3OUT |=  BIT0;      // Pull-up

    P3SEL |=  BIT1;      // Usar módulo dedicado
    P3REN |=  BIT1;      // Habilitar resistor
    P3OUT |=  BIT1;      // Pull-up

    UCB0CTL1 |= UCSWRST; // UCSI B0 em ressete

    UCB0CTL0 = UCSYNC |  //Síncrono
               UCMODE_3| //Modo I2C
               UCMST;    //Mestre

    UCB0BRW = BR100K;    //100 kbps
    UCB0CTL1 = UCSSEL_2; //SMCLK e remove ressete
}

// Configurar portas
void config_pinos(void){
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;
}

void delay(long limite){
    volatile long cont=0;
    while (cont++ < limite) ;
}
