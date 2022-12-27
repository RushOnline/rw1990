#include <OneWire.h>

/*
 * Управление производится через последовательный порт:
 * * "s" - поиск и чтение ключа. Ключ читается через раз (это глюк).
 * * "w" - запись ключа. Записываемый ключ задаётся переменной keyID.
 * ВНИМАНИЕ: Так как обнаружение ключа срабатывает через раз,
 *  чтобы записать ключ, перед тем как нажать "w", нужно добиться
 *  ответа "Not found" на команду "s".
 * 
 * ВНИМАНИЕ2:
 * * Проверена работа только на ключах RW-1990.1
 * * Подключение: центральный контакт гнезда подключен к iButtonPin и через резистор на 330 к Vcc.
 *      Также рекомендуют номиналы 1.5k - 2.2k. Но у меня работает на 330.
*/

#define iButtonPin  2   // Пин D11 для подлючения iButton (Data)
#define R_Led       10  // Номер контакта красного светодиода (запись?) - будем использовать светик в гнезде ключа.
#define B_Led       13  // Номер контакта голубого светодиода (TODO: для чего он?) - будем использовать светик arduino

typedef unsigned char byte;

enum emRWType
{
    TM01,       // неизвестный тип болванки, делается попытка записи TM-01 без финализации для dallas или c финализацией под cyfral или metacom
    RW1990_1,   // dallas-совместимые RW-1990, RW-1990.1, ТМ-08, ТМ-08v2
    RW1990_2,   // dallas-совместимая RW-1990.2
    TM2004      // dallas-совместимая TM2004 в доп. памятью 1кб
};

enum emKeyType {
    keyUnknown,
    keyDallas,
    keyTM2004,
    keyMetacom,
    keyCyfral
};

#define OLED_printError(...)
#define Sd_WriteStep(...)
#define Sd_ErrorBeep(...)
#define Sd_ReadOK(...)

emKeyType keyType;
OneWire ibutton (iButtonPin);
byte addr[8];

// byte keyID[8] = { 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F }; // "Универсальный" ключ. Прошивается последовательность 1:FF:FF:FF:FF:FF:FF:2F:CRC: 2F
// byte keyID[8] = { 0x01, 0xF0, 0x5C, 0xDF, 0x01, 0x00, 0x00, 0xCB};  // Aqua Key
byte keyID[8] = { 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x9B }; // Ko4eToBA 10k2 4n
byte halfT;

void BurnByte(byte data);
void BurnByteMC(byte buf[8]);
bool dataIsBurningOK(byte bitCnt);
byte calcAverage();

//*************** dallas **************
emRWType getRWtype()
{
    byte answer;

    // пробуем определить RW-1990.1
    ibutton.reset();
    ibutton.write(0xD1);  // проуем снять флаг записи для RW-1990.1
    ibutton.write_bit(1); // записываем значение флага записи = 1 - отключаем запись
    delay(10);
    pinMode(iButtonPin, INPUT);
    ibutton.reset();
    ibutton.write(0xB5); // send 0xB5 - запрос на чтение флага записи
    answer = ibutton.read();
    // Serial.print(F("\n Answer RW-1990.1: ")); Serial.println(answer, HEX);
    if (answer == 0xFE)
    {
        Serial.println(F(" Type: dallas RW-1990.1 "));
        return RW1990_1; // это RW-1990.1
    }
    // пробуем определить RW-1990.2
    ibutton.reset();
    ibutton.write(0x1D);  // пробуем установить флаг записи для RW-1990.2
    ibutton.write_bit(1); // записываем значение флага записи = 1 - включаем запись
    delay(10);
    pinMode(iButtonPin, INPUT);
    ibutton.reset();
    ibutton.write(0x1E); // send 0x1E - запрос на чтение флага записи
    answer = ibutton.read();
    if (answer == 0xFE)
    {
        ibutton.reset();
        ibutton.write(0x1D);  // возвращаем оратно запрет записи для RW-1990.2
        ibutton.write_bit(0); // записываем значение флага записи = 0 - выключаем запись
        delay(10);
        pinMode(iButtonPin, INPUT);
        Serial.println(F(" Type: dallas RW-1990.2 "));
        return RW1990_2; // это RW-1990.2
    }
    // пробуем определить TM-2004
    ibutton.reset();
    ibutton.write(0x33); // посылаем команду чтения ROM для перевода в расширенный 3-х байтовый режим
    for (byte i = 0; i < 8; i++)
        ibutton.read();  // читаем данные ключа
    ibutton.write(0xAA); // пробуем прочитать регистр статуса для TM-2004
    ibutton.write(0x00);
    ibutton.write(0x00);       // передаем адрес для считывания
    answer = ibutton.read();   // читаем CRC комманды и адреса
    byte m1[3] = {0xAA, 0, 0}; // вычисляем CRC комманды
    if (OneWire::crc8(m1, 3) == answer)
    {
        answer = ibutton.read(); // читаем регистр статуса
        // Serial.print(" status: "); Serial.println(answer, HEX);
        Serial.println(F(" Type: dallas TM2004"));
        ibutton.reset();
        return TM2004; // это Type: TM2004
    }
    ibutton.reset();
    Serial.println(F(" Type: dallas unknown, trying TM-01! "));
    return TM01; // это неизвестный тип DS1990, нужно перебирать алгоритмы записи (TM-01)
}

bool write2iBtnTM2004()
{ // функция записи на TM2004
    bool result = true;
    ibutton.reset();
    ibutton.write(0x3C); // команда записи ROM для TM-2004
    ibutton.write(0x00);
    ibutton.write(0x00); // передаем адрес с которого начинается запись
    for (byte i = 0; i < 8; i++)
    {
        digitalWrite(R_Led, !digitalRead(R_Led));
        ibutton.write(keyID[i]);
        ibutton.read();
        // answer = ibutton.read();
        // if (OneWire::crc8(m1, 3) != answer){result = false; break;} // crc не верный
        delayMicroseconds(600);
        ibutton.write_bit(1);
        delay(50); // испульс записи
        pinMode(iButtonPin, INPUT);
        Serial.print('*');
        Sd_WriteStep();
        if (keyID[i] != ibutton.read())
        {
            result = false;
            break;
        } // читаем записанный байт и сравниваем, с тем что должно записаться
    }
    if (!result)
    {
        ibutton.reset();
        Serial.println(F(" The key copy faild"));
        OLED_printError(F("The key copy faild"));
        Sd_ErrorBeep();
        digitalWrite(R_Led, HIGH);
        return false;
    }
    ibutton.reset();
    Serial.println(F(" The key has copied successesfully"));
    OLED_printError(F("The key has copied"), false);
    Sd_ReadOK();
    delay(2000);
    digitalWrite(R_Led, HIGH);
    return true;
}

bool write2iBtnRW1990_1_2_TM01(emRWType rwType)
{ // функция записи на RW1990.1, RW1990.2, TM-01C(F)
    byte rwCmd, bitCnt = 64, rwFlag = 1;
    switch (rwType)
    {
    case TM01:
        rwCmd = 0xC1;
        if ((keyType == keyMetacom) || (keyType == keyCyfral))
            bitCnt = 36;
        break; // TM-01C(F)
    case RW1990_1:
        rwCmd = 0xD1;
        rwFlag = 0;
        break; // RW1990.1 флаг записи инвертирован
    case RW1990_2:
        rwCmd = 0x1D;
        break; // RW1990.2
    default:
        Serial.print(F("invalid rwType: "));
        Serial.println((int)rwType);
    }
    ibutton.reset();
    ibutton.write(rwCmd);      // send 0xD1 - флаг записи
    ibutton.write_bit(rwFlag); // записываем значение флага записи = 1 - разрешить запись
    delay(5);
    pinMode(iButtonPin, INPUT);
    ibutton.reset();
    if (rwType == TM01)
        ibutton.write(0xC5);
    else
        ibutton.write(0xD5); // команда на запись
    if (bitCnt == 36)
        BurnByteMC(keyID);
    else
        for (byte i = 0; i < (bitCnt >> 3); i++)
        {
            digitalWrite(R_Led, !digitalRead(R_Led));
            if (rwType == RW1990_1)
                BurnByte(~keyID[i]); // запись происходит инверсно для RW1990.1
            else
                BurnByte(keyID[i]);
            Serial.print('*');
            Sd_WriteStep();
        }
    if (bitCnt == 64)
    {
        ibutton.write(rwCmd);       // send 0xD1 - флаг записи
        ibutton.write_bit(!rwFlag); // записываем значение флага записи = 1 - отключаем запись
        delay(5);
        pinMode(iButtonPin, INPUT);
    }
    digitalWrite(R_Led, LOW);
    if (!dataIsBurningOK(bitCnt))
    { // проверяем корректность записи
        Serial.println(F(" The key copy faild"));
        OLED_printError(F("The key copy faild"));
        Sd_ErrorBeep();
        digitalWrite(R_Led, HIGH);
        return false;
    }
    Serial.println(F(" The key has copied successesfully"));
    if ((keyType == keyMetacom) || (keyType == keyCyfral))
    { // переводим ключ из формата dallas
        ibutton.reset();
        if (keyType == keyCyfral)
            ibutton.write(0xCA); // send 0xCA - флаг финализации Cyfral
        else
            ibutton.write(0xCB); // send 0xCB - флаг финализации metacom
        ibutton.write_bit(1);    // записываем значение флага финализации = 1 - перевезти формат
        delay(10);
        pinMode(iButtonPin, INPUT);
    }
    Serial.println(F("The key has copied"));
    OLED_printError(F("The key has copied"), false);
    Sd_ReadOK();
    delay(2000);
    digitalWrite(R_Led, HIGH);
    return true;
}

void BurnByte(byte data)
{
    for (byte n_bit = 0; n_bit < 8; n_bit++)
    {
        ibutton.write_bit(data & 1);
        delay(5);         // даем время на прошивку каждого бита до 10 мс
        data = data >> 1; // переходим к следующему bit
    }
    pinMode(iButtonPin, INPUT);
}

void BurnByteMC(byte buf[8])
{
    byte j = 0;
    for (byte n_bit = 0; n_bit < 36; n_bit++)
    {
        ibutton.write_bit(((~buf[n_bit >> 3]) >> (7 - j)) & 1);
        delay(5); // даем время на прошивку каждого бита 5 мс
        j++;
        if (j > 7)
            j = 0;
    }
    pinMode(iButtonPin, INPUT);
}

void convetr2MC(byte buff[8])
{
    byte data;
    for (byte i = 0; i < 5; i++)
    {
        data = ~buff[i];
        buff[i] = 0;
        for (byte j = 0; j < 8; j++)
            if ((data >> j) & 1)
                bitSet(buff[i], 7 - j);
    }
    buff[4] &= 0xf0;
    buff[5] = 0;
    buff[6] = 0;
    buff[7] = 0;
}

bool dataIsBurningOK(byte bitCnt)
{
    byte buff[8];
    if (!ibutton.reset())
        return false;
    ibutton.write(0x33);
    ibutton.read_bytes(buff, 8);
    if (bitCnt == 36)
        convetr2MC(buff);
    byte Check = 0;
    for (byte i = 0; i < 8; i++)
    {
        if (keyID[i] == buff[i])
            Check++; // сравниваем код для записи с тем, что уже записано в ключе.
        Serial.print(buff[i], HEX);
        Serial.print(":");
    }
    if (Check != 8)
        return false; // если коды совпадают, ключ успешно скопирован
    return true;
}

bool write2iBtn()
{
    int Check = 0;
    if (!ibutton.search(addr))
    {
        ibutton.reset_search();
        Serial.println(F("Key not found."));
        return false;
    }
    Serial.print(F("The new key code is: "));
    for (byte i = 0; i < 8; i++)
    {
        Serial.print(addr[i], HEX);
        Serial.print(":");
        if (keyID[i] == addr[i])
            Check++; // сравниваем код для записи с тем, что уже записано в ключе.
    }
    if (Check == 8)
    { // если коды совпадают, ничего писать не нужно
        digitalWrite(R_Led, LOW);
        Serial.println(F(" it is the same key. Writing in not needed."));
        OLED_printError(F("It is the same key"));
        Sd_ErrorBeep();
        digitalWrite(R_Led, HIGH);
        delay(1000);
        return false;
    }
    emRWType rwType = getRWtype(); // определяем тип RW-1990.1 или 1990.2 или TM-01
    Serial.print(F("\n Burning iButton ID: "));
    if (rwType == TM2004)
        return write2iBtnTM2004(); // шьем TM2004
    else
        return write2iBtnRW1990_1_2_TM01(rwType); // пробуем прошить другие форматы
}

bool searchIbutton()
{
    if (!ibutton.search(addr))
    {
        ibutton.reset_search();
        delay(50);
        return false;
    }
    for (byte i = 0; i < 8; i++)
    {
        Serial.print(addr[i], HEX);
        Serial.print(":");
//        keyID[i] = addr[i]; // копируем прочтенный код в ReadID
    }
    if (addr[0] == 0x01)
    { // это ключ формата dallas
        keyType = keyDallas;
        if (getRWtype() == TM2004)
            keyType = keyTM2004;
        if (OneWire::crc8(addr, 7) != addr[7])
        {
            Serial.println(F("CRC is not valid!"));
            OLED_printError(F("CRC is not valid!"));
            Sd_ErrorBeep();
            digitalWrite(B_Led, HIGH);
            return false;
        }
        return true;
    }
    switch (addr[0] >> 4)
    {
    case 1:
        Serial.println(F(" Type: May be cyfral in dallas key"));
        break;
    case 2:
        Serial.println(F(" Type: May be metacom in dallas key"));
        break;
    case 3:
        Serial.println(F(" Type: unknown family dallas"));
        break;
    }
    keyType = keyUnknown;
    return true;
}

//************ Cyfral ***********************
unsigned long pulseACompA(bool pulse, byte Average = 80, unsigned long timeOut = 1500)
{ // pulse HIGH or LOW
    bool AcompState;
    unsigned long tEnd = micros() + timeOut;
    do
    {
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC))
            ; // Wait until the ADSC bit has been cleared
        if (ADCH > 200)
            return 0;
        if (ADCH > Average)
            AcompState = HIGH; // читаем флаг компаратора
        else
            AcompState = LOW;
        if (AcompState == pulse)
        {
            tEnd = micros() + timeOut;
            do
            {
                ADCSRA |= (1 << ADSC);
                while (ADCSRA & (1 << ADSC))
                    ; // Wait until the ADSC bit has been cleared
                if (ADCH > Average)
                    AcompState = HIGH; // читаем флаг компаратора
                else
                    AcompState = LOW;
                if (AcompState != pulse)
                    return (unsigned long)(micros() + timeOut - tEnd);
            } while (micros() < tEnd);
            return 0; // таймаут, импульс не вернуся оратно
        }             // end if
    } while (micros() < tEnd);
    return 0;
}

void ADCsetOn()
{
    ADMUX = (ADMUX & 0b11110000) | 0b0011 | (1 << ADLAR);               // (1 << REFS0); // подключаем к AC Линию A3 , левое выравние, измерение до Vcc
    ADCSRB = (ADCSRB & 0b11111000) | (1 << ACME);                       // источник перезапуска ADC FreeRun, включаем мультиплексор AC
    ADCSRA = (ADCSRA & 0b11111000) | 0b011 | (1 << ADEN) | (1 << ADSC); // | (1<<ADATE); // 0b011 делитель скорости ADC, // включаем ADC и запускаем ADC и autotriger ADC
}

void ACsetOn()
{
    ACSR |= 1 << ACBG;                     // Подключаем ко входу Ain0 1.1V для Cyfral/Metacom
    ADCSRA &= ~(1 << ADEN);                // выключаем ADC
    ADMUX = (ADMUX & 0b11110000) | 0b0011; // подключаем к AC Линию A3
    ADCSRB |= 1 << ACME;                   // включаем мультиплексор AC
}

bool read_cyfral(byte *buf, byte CyfralPin)
{
    unsigned long ti;
    byte i = 0, j = 0, k = 0;
    analogRead(iButtonPin);
    ADCsetOn();
    byte aver = calcAverage();
    unsigned long tEnd = millis() + 30;
    do
    {
        ti = pulseACompA(HIGH, aver);
        if ((ti == 0) || (ti > 260) || (ti < 10))
        {
            i = 0;
            j = 0;
            k = 0;
            continue;
        }
        if ((i < 3) && (ti > halfT))
        {
            i = 0;
            j = 0;
            k = 0;
            continue;
        } // контроль стартовой последовательности 0b0001
        if ((i == 3) && (ti < halfT))
            continue;
        if (ti > halfT)
            bitSet(buf[i >> 3], 7 - j);
        else if (i > 3)
            k++;
        if ((i > 3) && ((i - 3) % 4 == 0))
        { // начиная с 4-го бита проверяем количество нулей каждой строки из 4-и бит
            if (k != 1)
            {
                for (byte n = 0; n < (i >> 3) + 2; n++)
                    buf[n] = 0;
                i = 0;
                j = 0;
                k = 0;
                continue;
            } // если нулей больше одной - начинаем сначала
            k = 0;
        }
        j++;
        if (j > 7)
            j = 0;
        i++;
    } while ((millis() < tEnd) && (i < 36));
    if (i < 36)
        return false;
    return true;
}

bool searchCyfral()
{
    byte buf[8];
    for (byte i = 0; i < 8; i++)
    {
        addr[i] = 0;
        buf[i] = 0;
    }
    if (!read_cyfral(addr, iButtonPin))
        return false;
    if (!read_cyfral(buf, iButtonPin))
        return false;
    for (byte i = 0; i < 8; i++)
        if (addr[i] != buf[i])
            return false;
    keyType = keyCyfral;
    for (byte i = 0; i < 8; i++)
    {
        Serial.print(addr[i], HEX);
        Serial.print(":");
        //keyID[i] = addr[i]; // копируем прочтенный код в ReadID
    }
    Serial.println(F(" Type: Cyfral "));
    return true;
}

byte calcAverage()
{
    unsigned int sum = 127;
    byte preADCH = 0, j = 0;
    for (byte i = 0; i < 255; i++)
    {
        ADCSRA |= (1 << ADSC);
        delayMicroseconds(10);
        while (ADCSRA & (1 << ADSC))
            ; // Wait until the ADSC bit has been cleared
        sum += ADCH;
    }
    sum = sum >> 8;
    unsigned long tSt = micros();
    for (byte i = 0; i < 255; i++)
    {
        delayMicroseconds(4);
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC))
            ; // Wait until the ADSC bit has been cleared
        if (((ADCH > sum) && (preADCH < sum)) | ((ADCH < sum) && (preADCH > sum)))
        {
            j++;
            preADCH = ADCH;
        }
    }
    halfT = (byte)((micros() - tSt) / j);
    return (byte)sum;
}

bool read_metacom(byte *buf, byte MetacomPin)
{
    unsigned long ti;
    byte i = 0, j = 0, k = 0;
    analogRead(iButtonPin);
    ADCsetOn();
    byte aver = calcAverage();
    unsigned long tEnd = millis() + 30;
    do
    {
        ti = pulseACompA(LOW, aver);
        if ((ti == 0) || (ti > 500))
        {
            i = 0;
            j = 0;
            k = 0;
            continue;
        }
        if ((i == 0) && (ti + 30 < (unsigned long)(halfT << 1)))
            continue; // вычисляем период;
        if ((i == 2) && (ti > halfT))
        {
            i = 0;
            j = 0;
            continue;
        } // вычисляем период;
        if (((i == 1) || (i == 3)) && (ti < halfT))
        {
            i = 0;
            j = 0;
            continue;
        } // вычисляем период;
        if (ti < halfT)
        {
            bitSet(buf[i >> 3], 7 - j);
            if (i > 3)
                k++; // считаем кол-во единиц
        }
        if ((i > 3) && ((i - 3) % 8 == 0))
        { // начиная с 4-го бита проверяем контроль четности каждой строки из 8-и бит
            if (k & 1)
            {
                for (byte n = 0; n < (i >> 3) + 1; n++)
                    buf[n] = 0;
                i = 0;
                j = 0;
                k = 0;
                continue;
            } // если нечетно - начинаем сначала
            k = 0;
        }
        j++;
        if (j > 7)
            j = 0;
        i++;
    } while ((millis() < tEnd) && (i < 36));
    if (i < 36)
        return false;
    return true;
}

bool searchMetacom()
{
    byte buf[8];
    for (byte i = 0; i < 8; i++)
    {
        addr[i] = 0;
        buf[i] = 0;
    }
    if (!read_metacom(addr, iButtonPin))
        return false;
    if (!read_metacom(buf, iButtonPin))
        return false;
    for (byte i = 0; i < 8; i++)
        if (addr[i] != buf[i])
            return false;
    keyType = keyMetacom;
    for (byte i = 0; i < 8; i++)
    {
        Serial.print(addr[i], HEX);
        Serial.print(":");
        //keyID[i] = addr[i]; // копируем прочтенный код в ReadID
    }
    Serial.println(F(" Type: Metacom "));
    return true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ready");
}

void loop()
{
    auto cmd = Serial.read();
    switch (cmd)
    {
    case 'w':
        Serial.println(F("Writing..."));
        write2iBtn();
        break;
    case 's':
        Serial.println(F("Searching..."));
        if (!searchIbutton())
        {
            Serial.println("Not found.");
        }
        break;
    default:
        break;
    }
}