#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>


using namespace std ;



int main()
{
  vpImage<unsigned char> Ig(300,400,0);
  vpImage<unsigned char> Id(300,400,0);
  vpImage<vpRGBa> Iimage(876,1200);
  
 
  vpImageIo::read(Iimage,"../data/big-sleep.pgm") ;
  

  double L = 0.600 ;
  double l = 0.438;
  // Initialise the 3D coordinates of the Iimage corners
  vpColVector X[4];
  for (int i = 0; i < 4; i++) X[i].resize(3);
  // Top left corner
  X[0][0] = -L;
  X[0][1] = -l;
  X[0][2] = 0;
  
  // Top right corner
  X[1][0] = L;
  X[1][1] = -l;
  X[1][2] = 0;
  
  // Bottom right corner
  X[2][0] = L;
  X[2][1] = l;
  X[2][2] = 0;
  
  //Bottom left corner
  X[3][0] = -L;
  X[3][1] = l;
  X[3][2] = 0;
  


  vpImageSimulator sim;
  sim.init(Iimage, X);
  vpCameraParameters cam(800.0, 800.0, 200, 150);
  



  cam.printParameters() ;


  // I1g
  vpHomogeneousMatrix  gMo(0,0,2,  vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(gMo);
  sim.getImage(Ig,cam);
  cout << "Image I1g " <<endl ;
  cout << "gMo " << endl ;
  cout << gMo << endl ;

  // I1d décalage 10cm a droite de I1g
  /*vpHomogeneousMatrix dMo(-0.1,0,2, 
			  vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ; 
  // I1d décalage 20cm devant I1g
  vpHomogeneousMatrix dMo(0,0,1.8, 
			  vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ; */
  // I1d décalage origine
  vpHomogeneousMatrix dMo(0.1,0,1.9, 
			  vpMath::rad(5),vpMath::rad(5),vpMath::rad(5)) ; 
  sim.setCameraPosition(dMo);
  sim.getImage(Id,cam);  
  cout << "Image I1d " <<endl ;
  cout << "dMo " << endl ;
  cout << dMo << endl ;
 

  vpDisplayX dg(Ig,10,10,"Ig") ;
  vpDisplay::display(Ig) ;
  vpDisplay::flush(Ig) ;

  vpDisplayX dd(Id,10,10,"Id") ;
  vpDisplay::display(Id) ;
  vpDisplay::flush(Id) ;
    
   ////*POUR POINT DANS ID ET LIGNE DANS IG*////
    vpImagePoint pd ; 
  
  //récupération de K^-1 et sa transposée
  vpMatrix invK=cam.get_K_inverse();
  vpMatrix tInvK=invK.transpose();
  //récupération de la matrice de rotation dRg et de [dtg]
  vpHomogeneousMatrix gTd = gMo * dMo.inverse();
  vpRotationMatrix gRd; vpTranslationVector gttd;
  gTd.extract(gRd); gTd.extract(gttd);
  vpMatrix gtdSkew = gttd.skew();
  //calcul de la matrice fondamentale
  vpMatrix gFd = tInvK*(gtdSkew*(gRd*invK));
  

  for (int i=0 ; i < 5 ; i++)
    {
      cout << "Click point number " << i << endl ;
      vpDisplay::getClick(Id, pd) ;
      
      
      vpDisplay::displayCross(Id,pd,5,vpColor::red) ;


      // Calcul du lieu geometrique
      vpColVector vpd(3);
      vpd[0]=pd.get_u(); vpd[1]=pd.get_v(); vpd[2]=1;
      
      vpColVector line=gFd*vpd;
      
      int u1,v1,u2,v2;
      u1=0; 
      u2=Ig.getWidth(); 
      v1 = (-line[2]-line[0]*u1)/line[1];
      v2 = (-line[2]-line[0]*u2)/line[1];


      // Affichage dans Ig
      
      vpDisplay::displayLine(Ig,v1,u1,v2,u2,vpColor::lightGreen) ;

      vpDisplay::flush(Id) ;
      vpDisplay::flush(Ig) ;
    }
    
   ////*POUR POINT DANS IG ET LIGNE DANS ID*////
  /*vpImagePoint pg ; 
  
  //récupération de K^-1 et sa transposée
  vpMatrix invK=cam.get_K_inverse();
  vpMatrix tInvK=invK.transpose();
  //récupération de la matrice de rotation dRg et de [dtg]
  vpHomogeneousMatrix dTg = dMo * gMo.inverse();
  vpRotationMatrix dRg; vpTranslationVector dttg;
  dTg.extract(dRg); dTg.extract(dttg);
  vpMatrix dtgSkew = dttg.skew();
  //calcul de la matrice fondamentale
  vpMatrix dFg = tInvK*(dtgSkew*(dRg*invK));
  

  for (int i=0 ; i < 5 ; i++)
    {
      cout << "Click point number " << i << endl ;
      vpDisplay::getClick(Ig, pg) ;
      
      
      vpDisplay::displayCross(Ig,pg,5,vpColor::red) ;


      // Calcul du lieu geometrique
      vpColVector vpg(3);
      vpg[0]=pg.get_u(); vpg[1]=pg.get_v(); vpg[2]=1;
      
      vpColVector line=dFg*vpg;
      
      int u1,v1,u2,v2;
      u1=0; 
      u2=Id.getWidth(); 
      v1 = (-line[2]-line[0]*u1)/line[1];
      v2 = (-line[2]-line[0]*u2)/line[1];


      // Affichage dans Id
      
      vpDisplay::displayLine(Id,v1,u1,v2,u2,vpColor::lightGreen) ;

      vpDisplay::flush(Ig) ;
      vpDisplay::flush(Id) ;
    }*/

  // exemple de code pour sauvegarder une image avec les plan overlay
  vpImage<vpRGBa> IcolG, IcolD;
  vpDisplay::getImage(Ig,IcolG) ;
  vpDisplay::getImage(Id,IcolD);
  vpImageIo::write(IcolG,"I1g.pgm") ;
  vpImageIo::write(IcolD,"I1d.pgm") ;
  //vpImageIo::write(Ig,"I1g.pgm");

  


  vpDisplay::getClick(Id) ;
  cout << "OK " << endl ;

  vpDisplay::close(Id) ;
  vpDisplay::close(Ig) ;

  

  return 0;
}