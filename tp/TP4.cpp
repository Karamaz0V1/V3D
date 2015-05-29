#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpKeyPointSurf.h>
#include <visp/vpHomography.h>
#include <array>

#define NBPOINTS 17


using namespace std ;

void magic(array<vpImagePoint, NBPOINTS> & p1, array<vpImagePoint, NBPOINTS> & p2) {
    p1[0].set_u(117.5130997);   p1[0].set_v(62.34123611);   p2[0].set_u(202.841095);    p2[0].set_v(36.29648209);
    p1[1].set_u(84.06044006);   p1[1].set_v(67.55551147);   p2[1].set_u(169.5350189);   p2[1].set_v(26.80556679);
    p1[2].set_u(80.27194214);   p1[2].set_v(111.0672302);   p2[2].set_u(147.9641113);   p2[2].set_v(64.5475769);
    p1[3].set_u(342.6855164);   p1[3].set_v(199.8661346);   p2[3].set_u(63.4621048);    p2[3].set_v(68.28819275);
    p1[4].set_u(302.6676636);   p1[4].set_v(226.6687317);   p2[4].set_u(300.4017639);   p2[4].set_v(263.6835022);
    p1[5].set_u(101.5870972);   p1[5].set_v(63.0242424);    p2[5].set_u(187.8421478);   p2[5].set_v(29.56011963);
    p1[6].set_u(153.4119415);   p1[6].set_v(91.05652618);   p2[6].set_u(222.968277);    p2[6].set_v(77.2434845);
    p1[7].set_u(190.6780548);   p1[7].set_v(110.7231598);   p2[7].set_u(247.8312683);   p2[7].set_v(110.4263763);
    p1[8].set_u(302.8087463);   p1[8].set_v(133.9337616);   p2[8].set_u(339.9194641);   p2[8].set_v(178.880661);
    p1[9].set_u(162.7279968);   p1[9].set_v(276.4970398);   p2[9].set_u(152.7050171);   p2[9].set_v(248.9367065);
    p1[10].set_u(151.0850067);  p1[10].set_v(36.12360764);  p2[10].set_u(244.672287);   p2[10].set_v(25.44586563);
    p1[11].set_u(171.7740173);  p1[11].set_v(53.67162704);  p2[11].set_u(256.0083618);  p2[11].set_v(49.99362183);
    p1[12].set_u(116.7895355);  p1[12].set_v(74.19098663);  p2[12].set_u(196.8202972);  p2[12].set_v(45.97808456);
    p1[13].set_u(104.2023163);  p1[13].set_v(83.85998535);  p2[13].set_u(181.4200439);  p2[13].set_v(50.26084518);
    p1[14].set_u(84.71365356);  p1[14].set_v(190.8507233);  p2[14].set_u(300.4017639);  p2[14].set_v(263.6835022);
    p1[15].set_u(138.8526764);  p1[15].set_v(273.5761719);  p2[15].set_u(131.6974182);  p2[15].set_v(236.8515778);
    p1[16].set_u(167.2081451);  p1[16].set_v(96.59983063);  p2[16].set_u(233.1238556);  p2[16].set_v(88.96112061);
}

/*
void AfficheAppariement(
            vpImage<unsigned char> &I1,  
			vpImage<unsigned char> &I2, 
			vpImage<unsigned char> &I, 
			vpKeyPointSurf &surf)
    )
{

  for (int i =0 ; i < I1.getRows() ; i++)
    for (int j = 0 ; j < I1.getCols() ; j++)
      {
	I[i][j] = I1[i][j] ;
	I[i][j+I1.getCols()] = I2[i][j] ;
      }
  vpDisplay::display(I) ;

  vpDisplay::flush(I) ;
  int nb =  surf.matchPoint(I2);

  vpImagePoint p1[nb], p2[nb];
  
  
  for(unsigned int i=0; i<nb; i++)
    {
      surf.getMatchedPoints(i, p1[i], p2[i]);
      char s[10] ;
      sprintf(s,"%d",i) ;
      p2[i].set_u(p2[i].get_u()+I1.getCols()) ;
      vpDisplay::displayCharString(I,p1[i],s,vpColor::red) ;
      vpDisplay::displayCharString(I,p2[i],s,vpColor::red) ;
      vpDisplay::displayLine(I,p1[i],p2[i],vpColor::yellow) ;
      
    } 

  vpDisplay::flush(I) ;
}
//*/

// Calculer l'homographie aHb a partir des coordonnees des point p1 et p2
void DLT(unsigned int n, const vector<vpImagePoint> & p1, const vector<vpImagePoint> & p2, vpMatrix &H12)
{

#define NBPTMIN 4 
    if(n < NBPTMIN ) {
        cout << "there must be at least " << NBPTMIN <<  " points in the both images\n" <<endl  ;
        throw ;
    }
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


void random_my_indexes(std::vector<int> & indexes, int ind_max, int nb_ind) {
    for (int i = 0; i < nb_ind; i++) {
        int new_ind;
        bool notOk;
        do {
        new_ind = rand() % ind_max;
        notOk = false;
        for (int j = 0; j < i ; j++)
            if (indexes[j] == new_ind) { 
                notOk = true;
                break;
            }
        } while (notOk);
        indexes.push_back(new_ind);
    }
}

void make_my_points_array(const std::vector<int> & indexes, const std::array<vpImagePoint, NBPOINTS> & first_list, const std::array<vpImagePoint, NBPOINTS> second_list, vector<vpImagePoint> & first_array, vector<vpImagePoint> & second_array) {
    for (int i = 0; i < indexes.size(); i++) {
        first_array.push_back(first_list[indexes[i]]);
        second_array.push_back(second_list[indexes[i]]);
    }
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

vpImagePoint point_by_homography(const vpMatrix & H, const vpImagePoint & p1) {
    vpImagePoint p2;

    vpColVector p1m(3);
    pointToM(p1,p1m);

    vpColVector p2m = H * p1m;
    p2m /= p2m[2] ;

    MtoPoint(p2m, p2);
    return p2;
}

vpMatrix ransac_homography(const array<vpImagePoint, NBPOINTS> & p1, const array<vpImagePoint, NBPOINTS> & p2, int nb_try = 100, int nb_points_h = 5, float epsilon = 3) {
    int best_score = 0;
    vpMatrix best_homo;
    for (int x = 0; x < nb_try; x++) {
        std::vector<int> indexes;
        random_my_indexes(indexes, NBPOINTS, nb_points_h);
        vector<vpImagePoint> point1_h, point2_h;
        make_my_points_array(indexes, p1, p2, point1_h, point2_h);
        vpMatrix H12(3,3);
        DLT(nb_points_h, point1_h, point2_h, H12);

        int score = 0;
        for (int i = 0; i < NBPOINTS; i++ ) {
            vpImagePoint point_1_computed = point_by_homography(H12, p2[i]);
            if (abs(p1[i].get_u() - point_1_computed.get_u()) < epsilon) score ++;
        }

        if (score > best_score) {
            best_homo = H12;
            best_score = score;
        }
    }
    return best_homo;
}

void ransac_full(const array<vpImagePoint, NBPOINTS> & p1, const array<vpImagePoint, NBPOINTS> & p2, vector<vpImagePoint> & p1_correct, vector<vpImagePoint> & p2_correct, int nb_try = 100, int nb_points_h = 5, float epsilon = 3) {
    vpMatrix best_homo = ransac_homography(p1, p2, nb_try, nb_points_h, epsilon);

    for (int i = 0; i < NBPOINTS; i++) {
        vpImagePoint point_1_computed = point_by_homography(best_homo, p2[i]);
        if (abs(p1[i].get_u() - point_1_computed.get_u()) < epsilon) {
            p1_correct.push_back(p1[i]);
            p2_correct.push_back(p2[i]);
        }
    }
}
    
int main()
{
  vpImage<unsigned char> I1(300,400,0);
  vpImage<unsigned char> I2(300,400,0);
  vpImage<vpRGBa> Iimage(876,1200);
  
 
  vpImageIo::read(Iimage,"../data/big-sleep.jpg") ;
  

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
  vpHomogeneousMatrix  c1gMo(0,0,2,  vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(c1gMo);
  sim.getImage(I1,cam);
  cout << "Image I1g " <<endl ;
  cout << c1gMo << endl ;

  // I1d
  vpHomogeneousMatrix c1dMo(0.1,0,2, 
			    vpMath::rad(0),vpMath::rad(0),vpMath::rad(25)) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(c1dMo);
  sim.getImage(I2,cam);  
  cout << "Image I1d " <<endl ;
  cout << c1dMo << endl ;
 
  vpHomogeneousMatrix cgMcd = c1gMo * c1dMo.inverse() ;
  vpMatrix K = cam.get_K() ;

  vpDisplayX dg(I1,10,10,"I1") ;
  vpDisplay::display(I1) ;
  vpDisplay::flush(I1) ;

  vpDisplayX dd(I2,450,10,"I2") ;
  vpDisplay::display(I2) ;
  vpDisplay::flush(I2) ;

  // Image resultat
  vpImage<unsigned char> I(max(I1.getHeight(), I2.getHeight()), I1.getWidth() + I2.getWidth());

  for (int i = 0; i < I1.getHeight(); i++)
      for (int j = 0; j < I1.getWidth(); j++)
          I[i][j] = I1[i][j];

  for (int i = 0; i < I2.getHeight(); i++)
      for (int j = 0; j < I2.getWidth(); j++)
          I[i][j + I1.getWidth()] = I2[i][j];

  vpDisplayX d(I,10,400,"I") ;
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

  //vpKeyPointSurf surf;
  //surf.setDescriptorType(vpKeyPointSurf::extendedDescriptor) ; 	
  //First grab the reference image Irefrence
  //surf.setHessianThreshold(10000) ;

  //Build the reference SURF points.
  //surf.buildReference(I1);
  
  //AfficheAppariement(I1,  I2, I, surf) ;

  //Then grab another image which represents the current image Icurrent

  //Match points between the reference points and the SURF points computed in the current image.
  //int nb =  surf.matchPoint(I2);
  //cout << "Nombre de points mis en correspondance " <<  nb <<  endl ;
  //Display the matched points
  //surf.display(I1, I2);
  //vpImagePoint p1[nb], p2[nb];
  array<vpImagePoint, NBPOINTS> p1, p2;
  magic(p1, p2);

  int nb = NBPOINTS;
  if(nb >= 0){ // ... add paired points to vectPts
    
    for(unsigned int i=0; i<nb; i++)
    {
        //surf.getMatchedPoints(i, p1[i], p2[i]);
        char s[10] ;
        sprintf(s,"%d",i) ;
        cout << i <<"  "  << p1[i].get_u() <<"  " << p1[i].get_v() <<"  " ;
        cout <<  p2[i].get_u() <<"  " << p2[i].get_v() << endl;
        vpDisplay::displayCharString(I1,p1[i],s,vpColor::yellow) ;
        vpDisplay::displayCharString(I2,p2[i],s,vpColor::yellow) ;

        vpImagePoint p2_inline = vpImagePoint(p2[i].get_i(), p2[i].get_j() + I1.getWidth());
        vpDisplay::displayLine(I, p1[i], p2_inline, vpColor::red);
    }

  }

    vpDisplay::flush(I);
    vpDisplay::flush(I1);
    vpDisplay::flush(I2);

    vpImage<vpRGBa> Ic ;
    vpDisplay::getImage(I1,Ic) ;
    vpImageIo::write(Ic,"resultat_I1.jpg") ;
    vpDisplay::getImage(I2,Ic) ;
    vpImageIo::write(Ic,"resultat_I2.jpg") ;
    vpDisplay::getImage(I,Ic) ;
    vpImageIo::write(Ic,"resultat_I.jpg") ;

    std::cout<<"Waiting for click..."<<std::endl;
    vpDisplay::getClick(I);

    // --- RANSAC ---
    
    vector<vpImagePoint> ransac_p1, ransac_p2;
    ransac_full(p1, p2, ransac_p1, ransac_p2);

    for(unsigned int i=0; i<ransac_p1.size(); i++)
    {
        vpImagePoint p2_inline = vpImagePoint(ransac_p2[i].get_i(), ransac_p2[i].get_j() + I1.getWidth());
        vpDisplay::displayLine(I, ransac_p1[i], p2_inline, vpColor::green);
    }
    vpDisplay::flush(I);
    vpDisplay::getImage(I,Ic) ;
    vpImageIo::write(Ic,"resultat_ransac.jpg") ;
    vpDisplay::getClick(I);

  return 0;
}
