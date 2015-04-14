#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMatrix.h>


using namespace std ;

vpImage<unsigned char> rescale_value(vpImage<unsigned char> im1, vpImage<unsigned char> im2) {
   vpImage<unsigned char> tmp(144,192,0);
   }

int max(int a, int b) {
   if(a<b) return b;
   return a;
}

int min(int a, int b) {
   if(a<b) return a;
   return b;
}

void ewta(vpImage<unsigned char> Id, vpImage<unsigned char> Ig) {
  vpImage<float> distMap(144,192,0);
  vpImage<unsigned char> distMapC(144,192,0);
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
         distMap[el][jg]=jg-best;
         erreurMin=256;
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap.pgm") ;
}

void essd(vpImage<unsigned char> Id, vpImage<unsigned char> Ig) {
   vpImage<float> distMap(144,192,0);
   vpImage<unsigned char> distMapC(144,192,0);
   int sizeWindow;
   sizeWindow = 1;
   sizeWindow = 3;
   sizeWindow = 7;
   sizeWindow = 15;
   sizeWindow = 20;
   int m=sizeWindow/2;
   int erreurMin=65632;
   
   /*for(int i=0; i<m; i++) {
      for(int j=0; j<m; j++) {
         distMap[i][j]=0;
      }
   }
   for(int i=Ig.getHeight()-m; i<Ig.getHeight(); i++) {
      for(int j=Ig.getWidth()-m; j<Ig.getWidth(); j++) {
         distMap[i][j]=0;
      }
   }*/
   for(int i=m; i<Ig.getHeight()-m; i++) {
      for(int jg=m; jg<Ig.getWidth()-m; jg++) {              //pour chaque pixel
            int bestDisp;
            for(int jd=m; jd<Id.getWidth()-m;jd++) {             
               int erreur=0;
               for(int k=i-m; k<i+m; k++) {
                  for(int lg=jg-m; lg<jg+m; lg++) {          //pour chaque pixel de la fenêtre
                     int ld = lg + jg - jd;
                     erreur=erreur+pow(Id[k][ld]-Ig[k][lg],2);
                  }
               }
               if(erreur<erreurMin) {
                  erreurMin=erreur;
                  bestDisp=jd;
               }
            }
            distMap[i][jg]=jg-bestDisp;
            erreurMin=65632;
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap_SSD.pgm") ;
}

int main()
{
  vpImage<unsigned char> Ig(144,192,0);
  vpImage<unsigned char> Id(144,192,0);
  vpImageIo::read(Id,"../data/tsukuba-r.pgm") ;
  vpImageIo::read(Ig,"../data/tsukuba-l.pgm") ;
  
  ewta(Id,Ig);
  essd(Id,Ig);
}