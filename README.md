# BDX
Head Animation Code for static BDX

This code was designed to animate the head of BDX
3 Head Servos (L/R, Up/Dn, Tilt)
2 Ear Servos (L, R)
1 Neopixel for the head light
1 DF player for sound
2 240x240 eye displays connected together

Hardware 
2 Towerpro MG92B Servos for the "ears" (could probably use most non-continuous rotation micro servo)
3 35KG RDS3235 Servos for the head mechanism
1 DF Player mini (add whatever SD card you have an copy over the sounds folder) (https://wiki.dfrobot.com/dfplayer_mini_sku_dfr0299)
1 3 watt speaker
1 Xaio Seeed Studio ESP32C3 (https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/113991054/16652880)
1 USB cable for power
Assorted cables for inteconnection
2 HiLetGo 1.8 inch round displays (https://www.amazon.com/dp/B0CFFBW9M9)

Wiring From Xaio to other compoents - I run the setup from simple USB battery bank
D10 (GPIO10) for Displays SCLK
D9  (GPIO9)  for Displays MOSI
D8  (GPIO8)  for Displays DC1
D7  (GPI20)  for Displays CS1
D6  (GPIO21) for Tilt servo
D5  (GPIO7)  for Left/Right servo
D4  (GPIO6)  for Up/Down servo
D3  (GPIO5)  for TX to DFPlayer RX
D2  (GPIO4)  for Left Ear servo
D1  (GPIO3)  for Right Ear servo
D0  (GPIO2)  for NeoPixel LED
G   Ground   Common between all components
5V  Common between all components
3v Not used
