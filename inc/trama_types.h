#include <stdint.h>
class DATA_FRAME_TELEMETRYA{
    private:
       
        uint8_t         Header,
                        GyroXl,
                        GyroXm,
                        GyroYl,
                        GyroYm,
                        GyroZl,
                        GyroZm,
                        AccelXl,
                        AccelXm,
                        AccelYl, 
                        AccelYm,
                        AccelZl,
                        AccelZm,
                        Tail;
        static uint8_t  Trama[32];
        float           MagneX, MagneY, MagneZ;
        /*char            Lat0,Lat1,Lat2,Lat3,Lat4,Lat5,Lat6,Lat7,Lon0,Lon1,Lon2,Lon3,Lon4,Lon5,Lon6,Lon7,Lon8;*/
        
    public:
        void set_Header     (uint8_t sHead='A')             {Header=sHead;};
        void set_GyroXl     (uint8_t sGyroXl)           {GyroXl=sGyroXl;};
        void set_GyroXm     (uint8_t sGyroXm)           {GyroXm=sGyroXm;};
        void set_GyroYl     (uint8_t sGyroYl)           {GyroYl=sGyroYl;};
        void set_GyroYm     (uint8_t sGyroYm)           {GyroYm=sGyroYm;};
        void set_GyroZl     (uint8_t sGyroZl)           {GyroZl=sGyroZl;};
        void set_GyroZm     (uint8_t sGyroZm)           {GyroZm=sGyroZm;};
        void set_AccelXl    (uint8_t sAccelXl)          {AccelXl=sAccelXl;};
        void set_AccelXm    (uint8_t sAccelXm)          {AccelXm=sAccelXm;};
        void set_AccelYl    (uint8_t sAccelYl)          {AccelYl=sAccelYl;};
        void set_AccelYm    (uint8_t sAccelYm)          {AccelYm=sAccelYm;};
        void set_AccelZl    (uint8_t sAccelZl)          {AccelZl=sAccelZl;};
        void set_AccelZm    (uint8_t sAccelZm)          {AccelZm=sAccelZm;};
        void set_Magne      (float X, float Y, float Z) {MagneX=X;MagneY=Y; MagneZ=Z;};
        void set_trama(){
            Trama[0]=Header;
            Trama[1]=0;
            Trama[2]=GyroXl;
            Trama[3]=GyroXm;
            Trama[4]=GyroYl;
            Trama[5]=GyroXm;
            Trama[6]=GyroZl;
            Trama[7]=GyroZm;
            Trama[8]=AccelXl;
            Trama[9]=AccelXm;
            Trama[10]=AccelYl;
            Trama[11]=AccelYm;
            Trama[12]=AccelZl;
            Trama[13]=AccelZm;
            Trama[14]=0;
            Trama[15]= *(((uint32_t*)&MagneX)+0);
            Trama[19]= *(((uint32_t*)&MagneY)+0);
            Trama[23]= *(((uint32_t*)&MagneZ)+0);
            Trama[31]='Z';
        }
        uint8_t *GetTrama(){
            return Trama;
        }; 
};

class DATA_FRAME_TELEMETRYB{
    private:
        uint8_t Header;
        char Lat[8],Long[9],Trama[32];                   
    public:
        void set_Header     (uint8_t sHead='B')             {Header=sHead;};
        void set_Lat_long(char sLat[8],char sLong[9]){
            for(uint8_t i=0;i<8;i++) Lat[i]=sLat[i];
            for(uint8_t i;i<9;i++) Long[i]=sLong[i];
        };
        void set_trama(){
            Trama[0]=Header;
            Trama[1]=0;
            for(uint8_t i=2;i<8+2;i++) {Trama[i]=Lat[i-2]; Trama[i+8]=Long[i-2];}
            
            Trama[31]='Z';
        };
        char *GetTrama(){
            return Trama;
        } 
};