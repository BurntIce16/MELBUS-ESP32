# Volvo MELBUS ESP32

## Goals

- Clean up code and convert to a library for ease of use
- Get text functioning on screen
- Doccument protocol
- Dont fuck the wiring
- Get bluetooth audio streaming

## Resources

- <https://web.archive.org/web/20180217204549/http://volvo.wot.lv/wiki/doku.php?id=melbus>
- <https://gizmosnack.blogspot.com/2015/11/aux-in-volvo-hu-xxxx-radio.html?m=1>
- <https://gist.github.com/klalle/1ae1bfec5e2506918a3f89492180565e>
- <https://www.auxadapter.se/?page_id=712>
- <https://volvo.wot.lv/doku.php?id=melbus>

## Current Issues

- Pin defenitions are a bit fucky
  - Arduinos dont use gpio pins like esp32, intead they use pin ports like PIND which refers to pins 2-7 on an arduino
  - When you read from the PIND register in your Arduino code, you are essentially checking the state of the digital input pins connected to Port D. This can be useful for tasks such as reading switches or sensors connected to those pins.
  - Issue pins:
    - EIMSK
    - DDRD
    - PORTD
    - EIFR
    - EIMSK
- EEPROMClass has no member update
- A lot of the interupt logic uses bitbanging which isnt cross system compatible so im going to have to rewrite it to be more hardware agnostic
#   M E L B U S - E S P 3 2  
 