///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////main.cpp   main function of car tracking /////////////////////////////////////////////////////////////////////


//OpenCV library
//#include "cv.h"
//#include "cxcore.h"
//#include "highgui.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
//Debug���[�h�̏ꍇ
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
//Release���[�h�̏ꍇ
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <math.h>

//ORIGINAL header files
#include "Laser_func.h"
#include "car_det_func.h"
#include "Common.h"

//added by messi
#include <time.h>

#include "switch_float.h"

#include <string>
#include <fstream>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//definiton of functions//

int main(int argc, char* argv[]);	//main function (Object detection)

/////////////////////////////////////////
//data name
char ldata_name[]="2010_2_3.txt";		//laser data name
char mov_name[] = "out.avi";		//movie name

/////////////////////////////////////////
//window name
char WIN_A[]="CAR_TRACKING";		//laser data name
char WIN_B[]="2D-mapping";			//movie name

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FILE *resFP;

int main(int argc, char* argv[])
{
	FILE* fp;					//file pointer
	CvCapture *capt;			//movie file capture
	fpos_t curpos,fsize;		//file size
	int ss=0;					//image number
	int fnum=0;					//frame number
	bool FLAG = true;			//file end flag

    /* for measurement */
    struct timeval tv;
    struct timeval tv_1process_start, tv_1process_end;
    float one_process;

	//parameters
	FLOAT thresh = -0.5;												//threshold score of detection (default :0.0)
	FLOAT overlap = 0.4;												//threshold overlap parameter (default :0.4)
	FLOAT ratio = 1;													//resize ratio
	int TH_length = 80;													//tracking length threshold

    /* Output file for detect result */
    resFP = fopen("detect_result.dat", "w");

    double time_measure;
    time_measure = (double)cv::getTickCount();

	//get laser_save data pass (file should be in savedata folder)
	char *FPASS= get_file_pass(ldata_name);

	int i;
	printf("FPASS:");
	for (i=0;*(FPASS+i) != '\0';i++){
		printf("%c",*(FPASS+i));
	}
	printf("\n");

	//open save file
	if ((fp = fopen(FPASS,"rb")) == NULL) { printf("file open error!!\n"); exit(EXIT_FAILURE);}

	//get car-detector model
	MODEL *MO=load_model(ratio);

	//create lesult information
	RESULT *LR = create_result(0);

	//Particle filter information
	PINFO P_I = {0};

	//get file size and current file position
	get_f_size(fp,&curpos,&fsize);

	//open image-window(OpenCV)
	cvNamedWindow(WIN_A,CV_WINDOW_AUTOSIZE);	//for scan point mapping on image
	//cvNamedWindow(WIN_B,CV_WINDOW_AUTOSIZE);		//for scan point 2D mapping

	//get movie-file pass
	//char *MPASS= get_file_pass(mov_name);
	////load movie file
	//printf("%s\n",MPASS);
	//if((capt=cvCaptureFromAVI(MPASS))==NULL){printf("movie open error!!\n"); exit(EXIT_FAILURE);}

	////data skip
	//skip_data(fp,rapt,1200,&fnum);
	skip_data_2(fp,1,&ss);   //����ʒu����̉摜������(����50���ڂ���݂Ă���B)

    /* Open ImageSet file */
    std::ifstream ifs(argv[1]);
    if (ifs == 0)
      {
        fprintf(stderr, "ImageSetfile can't open\n");
        fprintf(stderr, "program terminated\n");
        return -1;
      }
    std::string line_contents;
    getline(ifs, line_contents); // skip first header line

	//load laser and movie data
	//for(int im=ss;im<2000;im++)

	/*
		added by messi
		timer
	*/

	clock_t start, end;

    //	for(int im=1;im<20;im++){
    while (getline(ifs, line_contents)) {

      /* load ImageSet */
      int image_id;
      int groundTruth;
      sscanf(line_contents.data(), "%06d %d", &image_id, &groundTruth);

      gettimeofday(&tv_1process_start, NULL);

		//IplImage *IMG;	//inputed-image
		////load movie
		//if((IMG=cvQueryFrame(capt))==NULL){printf("end of movie\n");break;}

		////create and resize 640x480 Image (for detection and tracking)
		//IplImage *IM_D=ipl_cre_resize(IMG,640,480);

		// load image
		IplImage *IM_D=load_suc_image(image_id);


		//printf("������%d\n",3 * IM_D -> width);
		//printf("��f%d\n",(int)(unsigned char)IM_D -> imageData[0]);
		//printf("��f%d\n",(int)(unsigned char)IM_D -> imageData[2*IM_D->widthStep]);
		//printf("��f%d\n",(int)(unsigned char)IM_D -> imageData[1920]);

		//for(int k=0;k<24;k++){printf("��f%d\n",(int)(unsigned char)IM_D -> imageData[k]);}

		///**HOG(����)*/
		//FLOAT **h;
		/////**h = new int[1000];*/
		//h = (FLOAT**)malloc(2*sizeof(FLOAT*));
		//h[0] =(FLOAT*) malloc(64*sizeof(FLOAT));
		//h[1] =(FLOAT*) malloc(64*sizeof(FLOAT));
		//int a=0;
		//for(int i=0;i<64;i+=0)
		//{
		//	for(int k=0;k<24;k+=3)
		//	{
		//		*(h[0]+i) = (int)(unsigned char)IM_D -> imageData[k+a*IM_D->widthStep];
		//		//printf("h;%d\n",*(h[0]+i));
		//		//printf("h;%d\n",(int)(unsigned char)IM_D -> imageData[k+a*IM_D->widthStep]);
		//		i++;
		//	}
		//		a++;
		//}
		//
		//FLOAT Dx,Dy,G, m[20];
		//long FLOAT l;
		//int z,z2;
		//for(z2=0;z2<20;z2++){m[z2]=0;}
		//for(int c=9;c<49;c+=8)
		//{
		//	for(int d=0;d<6;d++)
		//	{
		//	Dx = *(h[0]+c+d+1)-*(h[0]+c+d-1);
		//	Dy = *(h[0]+c+d+8)-*(h[0]+c+d-8);
		//	//printf("Dx%f\n",Dx);
		//	G= sqrt(pow(Dx,2.0)+pow(Dy,2.0));
		//	l = atan2((long FLOAT)Dy,(long FLOAT)Dx)/3.141592*180.0;
		//	//printf("�p�x%f\n",l);
		//	if(l>0)
		//	{
		//		for(z=0;z<9;z++)
		//			{
		//			if(z*20<= l && l<z*20+20)
		//				{
		//				*(h[1]+z) += G;
		//				//printf("�q�X�g�O����%f\n",*(h[1]+z));
		//				break;
		//				}
		//			}
		//	}
		//	else if(l<0)
		//	{
		//		for(z=9;z<18;z++)
		//			{
		//			if(z*20-360 <= l && l <z*20+20-360)
		//				{
		//				*(h[1]+z) += G;
		//				break;
		//				}
		//			}
		//	}
		//	}
		//}
		//for(int z=0;z<18;z++)
		//{
		//	printf("���z�@%f\n",*(h[1]+z));
		//}
		//delete [] h;
		//////////////////////////////////////
		///////////Car-Detection//////////////
		//////////////////////////////////////

		int D_NUMS=0;								//# of detected-object

		//IplImage *R_I = ipl_resize(IM_D,ratio);								//trime image for detection

		FLOAT *A_SCORE = ini_ac_score(IM_D);								//alloc accumulated score
		RESULT *CUR=car_detection(IM_D,MO,thresh,&D_NUMS,A_SCORE,overlap);	//detect car-boundary boxes
		//finalization(CUR,LR,&P_I,A_SCORE,R_I,TH_length);					//calculate tracking information

		//////////////////////////////////////
		//////////laser-radar fusion//////////
		//////////////////////////////////////

		////get scan-point data
		//SCANDATA *Sdata=get_s_data(fp,IM_D,&curpos);
		//radar_data_fusion(Sdata,IM_D,CUR,&P_I);									//draw scan point on image

		//////////////////////////////////////
		/////////////visualization////////////
		//////////////////////////////////////

		//Scan-point visualization
		//IplImage *TDMAP=draw_sdata(Sdata,IM_D,CUR);									//visualize scan data
		//detection result visualization
		show_rects(IM_D,CUR,ratio);											//visualize car-boundary-box
		//show_vector(IM_D,TDMAP,CUR,&P_I,ratio);									//visualize velocity-vector

		cvShowImage(WIN_A,IM_D);											//show image (RESULT & scan-point)
		//cvShowImage(WIN_B,TDMAP);											//show 2D scan-point MAP

		//update result
		//update_result(LR,CUR);												//update_result
        s_free(CUR);
        s_free(A_SCORE);
		//save result
		/*	IplImage **cimg = (IplImage **) cvAlloc (sizeof (IplImage *) * 2);
			IplImage *SaveIm = ini_Image(IM_D->width+480,480);*/
		/*char pass[255];
		  sprintf_s(pass,sizeof(pass),"save2\\%d.jpg",im);*/
		//cimg[0] = IM_D;
		//cimg[1] = TDMAP;
		//SaveIm = combine_image (2, cimg);
		//cvSaveImage(pass,SaveIm);

        gettimeofday(&tv_1process_end, NULL);
        tvsub(&tv_1process_end, &tv_1process_start, &tv);
        one_process = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

        printf("1process : %f\n", one_process);


		end = clock();
#if 0
		printf("end time is %d\n",end);
		printf("##################################################\n");
                printf("time is %d\n",end-start);
#endif
		start = 0;
		end = 0;
#if 1
        int IN_KEY=cvWaitKey(0);
        //        if(IN_KEY==0x1b) break;
        //        if(IN_KEY==1048603) // if 'Esc' key is typed
        if(IN_KEY=='\x1b') // if 'Esc' key is typed
          break;
#endif
		start = clock();
#if 0
		printf( "start time is %d\n", start);
#endif
		//release data
		//Release_sdata(Sdata);						//release scan point data

        //        R_I = NULL;
		cvReleaseImage(&IM_D);						//release detected image

		fnum++;
		printf("No %d\n",fnum);

	}

	//close window
	cvDestroyWindow(WIN_A);	//destroy window
	//cvDestroyWindow(WIN_B);	//destroy window

	//release car-detector-model
	free_model(MO);

	//release detection result
	release_result(LR);

	//close and release file information
	fclose(fp);			//close laser_file
	//cvReleaseCapture(&capt);

	s_free(FPASS);		//release laser_file pass
	//s_free(MPASS);		//release movie file pass


    time_measure = (double)cv::getTickCount() - time_measure;
    printf("execution time : %fms\n", time_measure*1000./cv::getTickFrequency());

    fprintf(resFP, "execution time : %fms\n", time_measure*1000./cv::getTickFrequency());
    fclose(resFP);

	return 0;
}
