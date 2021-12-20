#ifdef DEBUG
    #define DPRINT(x) Serial.print(x)
    #define DPRINTF(x, y) Serial.print(x, y)
    #define DPRINTLN(x) Serial.println(x)
    #define DPRINTLNF(x, y) Serial.println(x, y)
    #define IFD if(true)
#else 
    #define DPRINT(x)
    #define DPRINTF(x, y)
    #define DPRINTLN(x)
    #define DPRINTLNF(x, y)
    #define IFD if(false)
#endif