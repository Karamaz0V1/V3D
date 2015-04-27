#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot.h>


using namespace std ;

// Calculer l'homographie aHb a partir des coordonnees des point p1 et p2
void DLT(unsigned int n,
	 vpImagePoint *p1,
	 vpImagePoint *p2,
	 vpMatrix &H12)
{

    // NBPTMIN points son necessaire ; remplace le 1e6 par ce nombre
#define NBPTMIN 3 
    if(n<    NBPTMIN )
    {
    cout << "there must be at least " << NBPTMIN <<  " points in the both images\n" <<endl  ;
    throw ;
    }
    cout<<p1[0].get_u()<<" "<<p1[0].get_v()<<endl;
    cout<<p2[0].get_u()<<" "<<p2[0].get_v()<<endl;

    vpMatrix gamma(2*n,9);

    for (int i=0; i<n; i++) {
        gamma[i*2][0] = 0;
        gamma[i*2][1] = 0; 
        gamma[i*2][2] = 0; 

        gamma[i*2][3] = -1 * p1[i].get_u(); 
        gamma[i*2][4] = -1 * p1[i].get_v(); 
        gamma[i*2][5] = -1; 

        gamma[i*2][6] = p2[i].get_v() * p1[i].get_u(); 
        gamma[i*2][7] = p2[i].get_v() * p1[i].get_v(); 
        gamma[i*2][8] = p2[i].get_v(); 

        gamma[i*2+1][0] = p1[i].get_u(); 
        gamma[i*2+1][1] = p1[i].get_v(); 
        gamma[i*2+1][2] = 1; 

        gamma[i*2+1][3] = 0;
        gamma[i*2+1][4] = 0; 
        gamma[i*2+1][5] = 0; 

        gamma[i*2+1][6] = -p2[i].get_u() * p1[i].get_u(); 
        gamma[i*2+1][7] = -p2[i].get_u() * p1[i].get_v(); 
        gamma[i*2+1][8] = -p2[i].get_u(); 
    }

    std::cout<<gamma<<std::endl;
    vpMatrix v;
    vpColVector d;
    gamma.svd(d, v);
    std::cout<<"V : \n"<<v<<std::endl;
    std::cout<<"D : \n"<<d<<std::endl;

//    for (int i=0; i<9; i++) {
//        float min = v[i][0];
//        int jmin = 0;
//        for (int j=1; j<9; j++)
//            if (v[i][j] < min) { jmin = j; min = v[i][j];}

    H12[0][0] = v[0][8] / v[8][8];
    H12[0][1] = v[1][8] / v[8][8];
    H12[0][2] = v[2][8] / v[8][8];
    H12[1][0] = v[3][8] / v[8][8];
    H12[1][1] = v[4][8] / v[8][8];
    H12[1][2] = v[5][8] / v[8][8];
    H12[2][0] = v[6][8] / v[8][8];
    H12[2][1] = v[7][8] / v[8][8];
    H12[2][2] = v[8][8] / v[8][8];
    vpMatrix H21 = H12.inverseByLU();
    H12 = H21;
}

void pointToM(const vpImagePoint & point, vpColVector & pointm) {
    pointm[0] = point.get_u();
    pointm[1] = point.get_v();
    pointm[2] = 1;
}

void MtoPoint(const vpColVector & pointm, vpImagePoint & point) {
    point.set_u(pointm[0]);
    point.set_v(pointm[1]);
}

int main()
{
  vpImage<unsigned char> I1;
  vpImage<unsigned char> I2;
  vpImage<vpRGBa> Iimage(876,1200);
  
 
  vpImageIo::read(I1,"../data/I1.pgm") ;
  vpImageIo::read(I2,"../data/I2.pgm") ;



  vpCameraParameters cam(800.0, 800.0, 200, 150);
  cam.printParameters() ;

  vpDisplayX d1(I1,10,10,"I1") ;
  vpDisplay::display(I1) ;
  vpDisplay::flush(I1) ;

  vpDisplayX d2(I2,450,10,"I2") ;
  vpDisplay::display(I2) ;
  vpDisplay::flush(I2) ;

  int nb = 5;
  vpImagePoint p1[5], p2[5]; // 5 = nb
  
  // clicker 5 point sur l'image I2 ; recuperer leur coordonnees
  for(unsigned int i=0; i<nb; i++)
    {
      vpDisplay::getClick(I1, p1[i]) ;
      vpDot d ;
      d.initTracking(I1,p1[i]) ;
      d.track(I1,p1[i]) ;
      char s[10] ;
      sprintf(s,"%d",i) ;
      vpDisplay::displayCross(I1,p1[i],10,vpColor::blue) ;
      vpDisplay::displayCharString(I1,p1[i],s,vpColor::red) ;
      vpDisplay::flush(I1) ;
    }
  
  // clicker 5 point sur l'image I1 ; recuperer leur coordonnees
  // faites attention a les clicker dans le meme ordre
  for(unsigned int i=0; i<nb; i++)
    {
      vpDisplay::getClick(I2, p2[i]) ;
      vpDot d ;
      d.initTracking(I2,p2[i]) ;
      d.track(I2,p2[i]) ;
      char s[10] ;
      sprintf(s,"%d",i) ;
      vpDisplay::displayCross(I2,p2[i],10,vpColor::green) ;
      vpDisplay::displayCharString(I2,p2[i],s,vpColor::red) ;
      vpDisplay::flush(I2) ;
    }
  

  // Calculer l'homographie
  vpMatrix H12(3,3) ;
  DLT(nb,p1, p2, H12) ;

  cout << "Homographie H12 : " <<endl ; 
  cout << H12 << endl ;

  //Verification 
  double residue =0 ;
  for (int i=0 ; i < nb ; i++) 
    {
      // Connaissant le formule permettant le transfert des points p2 dans p1
      // Calculer les coordonnées des point p1 connaissant p2 et dHg
      vpImagePoint p1_calcule  ;

      vpColVector p2m(3);
      pointToM(p2[i],p2m);
      vpColVector p1m = H12 * p2m;
      
      p1m /= p1m[2] ;

      MtoPoint(p1m, p1_calcule);

      
      //    p1_calcule  = ... ;

      // en deduire l'erreur sur commise sur chaque point et 
      // afficher un cercle de rayon 10 fois cette erreur
      double r ;
      //      r = ... ;
      r = vpImagePoint::distance(p1_calcule, p1[i]);
      //cout << "point " <<i << "  " << r <<endl ;;
      double rayon ;
      rayon = sqrt(r)*10 ; if (rayon < 10) rayon =10 ;
      vpDisplay::displayCircle(I1,p1_calcule,rayon,vpColor::green) ; ;
    }


  vpDisplay::flush(I1) ;
  vpImage<vpRGBa> Ic ;
  vpDisplay::getImage(I1,Ic) ;
  vpImageIo::write(Ic,"resultat.jpg") ;

  vpDisplay::getClick(I1) ;

  vpDisplay::close(I2) ;
  vpDisplay::close(I1) ;

  vpImage<unsigned char> Ir(700,700);
  for (int i=0; i<Ir.getHeight(); i++)
      for (int j=0; j<Ir.getWidth(); j++) {
          if (i<I1.getHeight() && j<I1.getWidth()) Ir[i][j] = I1[i][j] / 2;
          vpColVector p2(3);
          p2[0] = j;
          p2[1] = i;
          p2[2] = 1;
          vpMatrix H21 = H12.inverseByLU();
          vpColVector pr = H21 * p2;
          pr /= pr[2];
          int i2 = pr[1];
          int j2 = pr[0];
          if (i2<I2.getHeight() && j2<I2.getWidth()) Ir[i][j] += I2[i2][j2] / 2;
      }

  vpDisplayX d3(Ir,450,10,"Ir") ;
  vpDisplay::display(Ir);
  vpDisplay::flush(Ir) ;
  vpDisplay::getClick(Ir) ;
  vpDisplay::close(Ir) ;

  return 0;
}
