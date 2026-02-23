# include <Arduino.h>
class timer{
    private:
        unsigned long long count=0;
    public:
        void update(int num){
            if ( millis() - count >= num){
                count = millis();
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
        }
};