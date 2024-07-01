/* We're actually trying to accomplish the same scenario with multiple sensors measuring in parallel. According to the core API documentation from ST, It's possible to do so using that ST API's method call you posted: VL53L0X_SetDeviceAddress(). But the dynamic address assignment for each of the many sensors must occur by turning them on one by one via XSHUT PIN (SHDN PIN available on the Adafruit's breakout) for the sensor (turned on by default if using the adafruit's modules). As long as only one module exists with ON state using the default address, 0x29, at a given time during initialization, it seems you'll be able to set the new address for that one using the API's call; then turn ON the next module and do the setting of its new address, an so on until having them all initialized with their own unique address as slaves on the I2C bus. Make sure you turn all the modules OFF by signaling their SHDN PIN low before start programing them, then wake them one by one and set the new address using the call for the one module that was turned on, and so on...

We are currently writing our own version of the API but handling the instantiation of multiple VL53L0X in a configurable way. We can't share it yet, but you can take a look at this ST API's documentation explaining how to address the multiple sensors scenario:

http://www.st.com/content/ccc/resource/ ... 280486.pdf

The documentation from ST above, mentions the use of GPIO expander for the cases where your Arduino board does not have enough PINs to handle either the ON signal via XSHUT PIN or the interruption coming from the modules when data is present. If you have enough pins on your Arduino you won't need the expanders


https://www.st.com/content/ccc/resource/technical/document/application_note/group0/0e/0a/96/1b/82/19/4f/c2/DM00280486/files/DM00280486.pdf/jcr:content/translations/en.DM00280486.pdf
*/