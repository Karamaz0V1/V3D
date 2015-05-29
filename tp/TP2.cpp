#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMatrix.h>


using namespace std ;


//TODO à revoir, sait-on jamais
void ewta(vpImage<unsigned char> Id, vpImage<unsigned char> Ig) {
  vpImage<float> distMap(Id.getHeight(),Id.getWidth(),0);
  vpImage<unsigned char> distMapC(Id.getHeight(),Id.getWidth(),0);
  int erreurMin=256;
  for(int el=0; el<Ig.getHeight(); el++) {            //pour chaque ligne épipolaire
      for(int jg=0; jg<Ig.getWidth(); jg++) {         //pour chaque pixel de la ligne courante dans l'image de gauche
         int best;
         for(int jd=0; jd<Id.getWidth(); jd++) {      //pour chaque pixel de la même ligne dans l'image de droite
            int erreur = abs(Id[el][jd]-Ig[el][jg]);
            if(erreur<erreurMin) {
               erreurMin=erreur;
               best=jd;
            }
         }
         distMap[el][jg]=best-jg;
         erreurMin=256;
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap.pgm") ;
}

/*
void essd(vpImage<unsigned char> Id, vpImage<unsigned char> Ig,int sizeWindow) {
   vpImage<float> distMap(Id.getHeight(),Id.getWidth(),0);
   vpImage<unsigned char> distMapC(Id.getHeight(),Id.getWidth(),0);
   int m=sizeWindow/2;
   int erreurMin=65632;
   
   for(int i=m; i<Ig.getHeight()-m; i++) {
      for(int jg=m; jg<Ig.getWidth()-m; jg++) {              //pour chaque pixel
            int bestDisp;
            for(int jd=m; jd<Id.getWidth()-m;jd++) {             
               int erreur=0;
               for(int k=i-m; k<i+m; k++) {
                  for(int lg=jg-m; lg<jg+m; lg++) {          //pour chaque pixel de la fenêtre
                     int ld = lg + jd - jg;
                     erreur=erreur+pow(Id[k][lg]-Ig[k][ld],2);
                  }
               }
               if(erreur<erreurMin) {
                  erreurMin=erreur;
                  bestDisp=jd;
               }
            }
            distMap[i][jg]=bestDisp-jg;
            erreurMin=65632;
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap_SSD.pgm") ;
}
*/

void essd(vpImage<unsigned char> Id, vpImage<unsigned char> Ig,int sizeWindow) {
   int med = sizeWindow/2;
   vpImage<float> distMap(Id.getHeight(),Id.getWidth(),0);
   vpImage<unsigned char> distMapC(Id.getHeight(),Id.getWidth(),0);
   
   //Création du masque
   
   vpImage<double> masque(sizeWindow,sizeWindow,1);
   if(sizeWindow == 3) {
      masque[0][0]= 1;
      masque[0][1]= 2;
      masque[0][2]= 1;
      masque[1][0]= 2;
      masque[1][1]= 4;
      masque[1][2]= 2;
      masque[2][0]= 1;
      masque[2][1]= 2;
      masque[2][2]= 1;
   } 
   
   for(int i = 0; i < Ig.getHeight(); i++) {
      for(int jg = 0; jg < Ig.getWidth(); jg++) {      //pour chaque pixel de l'image Ig
         float erreurMin = 65632;
         int a = 0;
         float val = 0.0;
         for(int jd = 0; jd < Id.getWidth(); jd++) {        //sur toute la largeur de l'image Id
            val = 0.0;
            if(masque.getWidth()==1)
               val = (Id[i][jd] - Ig[i][jg]) * (Id[i][jd] - Ig[i][jg]);
            else {
               for(int h = 0; h < masque.getHeight(); h++) {
                  for(int w = 0; w < masque.getWidth(); w++) {
                     val = val + masque[h][w] * (Id[i-med+h][jd-med+w] - Ig[i-med+h][jg-med+w]) * (Id[i-med+h][jd-med+w] - Ig[i-med+h][jg-med+w]);
                  }
               }
            }
            if(val<erreurMin) {
               erreurMin = val;
               if(abs(jd-jg) > 255)
                  distMap[i][jg] = 0;
               else
                  distMap[i][jg] = abs(jd-jg);
            }
         }
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap_SSD.pgm") ;
}
   
   

void distMap(vpImage<unsigned char> Id,vpImage<unsigned char> Ig) {
   //ewta(Id,Ig);
   int sizeWindow=1;
   sizeWindow = 3;
   sizeWindow = 7;
   //sizeWindow = 15;
   //sizeWindow = 20;
   essd(Id,Ig,sizeWindow);
}

int main()
{
  vpImage<unsigned char> Ig(144,192,0);
  vpImage<unsigned char> Id(144,192,0);
  vpImageIo::read(Id,"../data/tsukuba-r.pgm") ;
  vpImageIo::read(Ig,"../data/tsukuba-l.pgm") ;  
  distMap(Id,Ig);
}