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
         distMap[el][jg]=max(0,min(255,abs(jg-best)));
         erreurMin=256;
      }
   }
   vpImageConvert::convert(distMap,distMapC);
   vpImageIo::write(distMapC,"tsubuka_distMap.pgm") ;
}
void essd(vpImage<unsigned char> Id, vpImage<unsigned char> Ig) {
   vpImage<float> distMap(144,192,0);
   vpImage<unsigned char> distMapC(144,192,0);
   
   for(int i=0; i<Ig.getHeight; i++) {
      for(int j=0; j<Ig.getWidth; j++) {              //pour chaque pixel
         for(disp=0; disp<256; disp++) {               //pour chaque disparité
            for(k=0; k<sizeWindow; k++) {
               for(l=0; l<sizeWindow; l++) {          //pour chaque pixel de la fenêtre
                  erreur=erreur+pow(Ig[,2);
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