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
                        Tail,
                        MagneXl,
                        MagneXm,
                        MagneYl,
                        MagneYm,
                        MagneZl,
                        MagneZm;
        /*char            Lat0,Lat1,Lat2,Lat3,Lat4,Lat5,Lat6,Lat7,Lon0,Lon1,Lon2,Lon3,Lon4,Lon5,Lon6,Lon7,Lon8;*/
        
    public:
        DATA_FRAME_TELEMETRYA(){};
        ~DATA_FRAME_TELEMETRYA(){};
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
        void set_MagneX     (uint8_t l, uint8_t m) {MagneXl=l;MagneXm=m;};
        void set_MagneY     (uint8_t l, uint8_t m) {MagneYl=l;MagneYm=m;};
        void set_MagneZ     (uint8_t l, uint8_t m) {MagneZl=l;MagneZm=m;};
        
        char *GetTrama(){
            static char  Trama[32];
                Trama[0]=Header;
                Trama[1]=0;
                Trama[2]=GyroXl;
                Trama[3]=GyroXm;
                Trama[4]=GyroYl;
                Trama[5]=GyroYm;
                Trama[6]=GyroZl;
                Trama[7]=GyroZm;
                Trama[8]=0;
                Trama[9]= AccelXl;
                Trama[10]=AccelXm;
                Trama[11]=AccelYl;
                Trama[12]=AccelYm;
                Trama[13]=AccelZl;
                Trama[14]=AccelZm;
                Trama[15]=0;
                Trama[16]=MagneXl;
                Trama[17]=MagneXm;
                Trama[18]=MagneYl;
                Trama[19]=MagneYm;
                Trama[20]=MagneZl;
                Trama[21]=MagneZm;
                Trama[22]=0; 
                Trama[31]='Z';
            return Trama;
        };
        void RXtrama(char rTrama[32]){
            Header=rTrama[0];
            GyroXl=rTrama[2];
            GyroXm=rTrama[3];
            GyroYl=rTrama[4];
            GyroYm=rTrama[5];
            GyroZl=rTrama[6];
            GyroZm=rTrama[7];
            AccelXl=rTrama[9];
            AccelXm=rTrama[10];
            AccelYl=rTrama[11];
            AccelYm=rTrama[12];
            AccelZl=rTrama[13];
            AccelZm=rTrama[14];
            
            MagneXl=rTrama[16];
            MagneXm=rTrama[17];
            MagneYl=rTrama[18];
            MagneYm=rTrama[19];
            MagneZl=rTrama[20];
            MagneZm=rTrama[21];
            
            Tail=rTrama[31];
        } 
};



class DATA_FRAME_TELEMETRYB{
    private:
        uint8_t Header, Vel, Dirl,Dirm, Tail;
        char Lat[8],Long[9];                   
    public:
        void set_Header     (uint8_t sHead='B')             {Header=sHead;};
        void set_Lat(char sLat[8]){
            for(uint8_t i=2;i<10;i++) Lat[i-2]=sLat[i-2];};
        void set_Lon(char sLong[9]){
            for(uint8_t i=11;i<9+11;i++) Long[i-11]=sLong[i-11];
        };
        void set_WindVel(uint8_t V){
            Vel=V;
        }
        void set_WindDir(uint8_t sDirl,uint8_t sDirm){
            Dirl=sDirl;Dirm=sDirm;
        }
        
        char *GetTrama(){
            static char Trama[32];
            Trama[0]=Header;
            Trama[1]=0;
            for(uint8_t i=2;i<8+2;i++) Trama[i]=Lat[i-2]; 
            for(uint8_t i=11;i<9+11;i++) Trama[i]=Long[i-11];
            Trama[26]=0;
            Trama[27]=Vel;
            Trama[28]=Dirl;
            Trama[29]=Dirm;
            Trama[30]=0;
            
            Trama[31]='Z';
            return Trama;
        };

        void RXtrama(char rTrama[32]){
            Header=rTrama[0];
            for(uint8_t i=2;i<8+2;i++) Lat[i-2]=rTrama[i];
            for(uint8_t i=11;i<9+11;i++) Long[i-11]=rTrama[i];
            
            Vel=rTrama[27];
            Dirl=rTrama[28];
            Dirm=rTrama[29];
            Tail=rTrama[31];
        } 
};

