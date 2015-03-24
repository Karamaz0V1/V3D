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

  // I1d
  vpHomogeneousMatrix dMo(0.1,0,2, 
			  vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ; 

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

  vpImagePoint pd ; 
  
      vpMatrix k=cam.get_K();
      vpMatrix invK=cam.get_K_inverse();
      vpMatrix transInvK=invK.transpose();
      vpHomogeneousMatrix dTg = dMo*gMo.inverse();
      vpRotationMatrix dRg;
      dTg.extract(dRg);
      vpTranslationVector dttg;
      dTg.extract(dttg);
      vpMatrix dtgSk = dttg.skew();
      vpMatrix dfg=transInvK*(dtgSk*(dRg*invK));
      vpMatrix fdg=dfg.inverseByQR(); //TODO
      vpImagePoint pg;

  for (int i=0 ; i < 5 ; i++)
    {
      cout << "Click point number " << i << endl ;
      vpDisplay::getClick(Id, pd) ;
      
      
      vpDisplay::displayCross(Id,pd,5,vpColor::red) ;
      
      int u1 = pd.get_u();
      int v1 = pd.get_v();
      vpColVector vpd(3);
      vpd[0]=u1;
      vpd[1]=v1;
      vpd[2]=1;
      u1=0;
      
      vpColVector line = dfg*vpd;
      
      
      int u2 = Ig.getWidth();
      int v2 = v1;
      
      vpDisplay::displayLine(Ig,v1,u1,v2,u2,vpColor::lightGreen);
      
      // Calcul du lieu geometrique
      //....


      // Affichage dans Ig
      
      //      vpDisplay::displayXXXX(Ig,...) ;

      vpDisplay::flush(Id) ;
      vpDisplay::flush(Ig) ;
    }

  // exemple de code pour sauvegarder une image avec les plan overlay
  vpImage<vpRGBa> Icol ;
  vpDisplay::getImage(Ig,Icol) ;
  vpImageIo::write(Icol,"resultat.pgm") ;
  vpImageIo::write(Ig,"I1g.pgm") ;

  


  vpDisplay::getClick(Id) ;
  cout << "OK " << endl ;

  vpDisplay::close(Id) ;
  vpDisplay::close(Ig) ;

  

  return 0;
}