/*
autoCalib.cpp
Automatic Camera calibration with Viam and OpenCV
David Marquez-Gamez
LAAS-CNRS
dmarquez@laas.fr

Compiling with gcc:
g++ autoCalib.cpp -o autoCalib -I/usr/local/openrobots/include/opencv -I/usr/local/openrobots/include -I/usr/local/openrobots/include/viam -L/usr/local/openrobots/lib -lm -lcv -lhighgui -lcvaux -lcxcore -lviam

RINAVEC-project
*/

#include <stdio.h>
#include <cv.h>
#include <viamlib.h>
#include <viamcv.h>
#include <iostream>
#include <sstream>
#include <string>
#include <highgui.h> 

using namespace std;

int main(int argc, char* argv[]){

    int n_boards = 0;
    const int board_dt = 20;
    int board_w;
    int board_h;

    printf("Init fw camera\n");

    int r;
    viam_bank_t bank;
    viam_handle_t handle;

    //flea2
    //const std::string camera_id = "0x00b09d01006fb38f";
    const std::string camera_id = "0x00b09d010063565b";
    viam_hwmode_t hwmode = { VIAM_HWSZ_640x480, VIAM_HWFMT_MONO8, VIAM_HW_FIXED, VIAM_HWFPS_30, VIAM_HWTRIGGER_INTERNAL };
    int w = 640; int h = 480;

//     Micron IR                      "0x00b09d01006fb38f";
//    const std::string camera_id = "0x0001687300025581";
//    viam_hwmode_t hwmode = { VIAM_HWSZ_164x129, VIAM_HWFMT_MONO8, VIAM_HW_FIXED, VIAM_HWFPS_30, VIAM_HWTRIGGER_INTERNAL };
//    int w = 164; int h = 129;

    handle = viam_init();
    viam_camera_t camera = viam_camera_create(handle, camera_id.c_str(), "camera1");
    bank = viam_bank_create(handle,"bank1");
    r = viam_bank_cameraadd(handle,bank,camera,"image1");
    r = viam_camera_sethwmode(handle, camera, &hwmode);
    r = viam_hardware_load(handle,"dc1394");
    r = viam_hardware_attach(handle);
    r = viam_bank_configure(handle, bank);
    r = viam_datatransmit(handle, bank, VIAM_ON);

    int o;

    board_w = 5; // Board width in squares
    board_h = 8; // Board height
    n_boards = 8; // Number of boards
    int board_n = board_w * board_h;
    CvSize board_sz = cvSize( board_w, board_h );
    CvCapture* capture = cvCreateCameraCapture( 0 );
    assert( capture );
    

    //cvNamedWindow( "Acquisition" );
    cvNamedWindow( "Calibration" );
    // Allocate Sotrage
    CvMat* image_points       = cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
    CvMat* object_points      = cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
    CvMat* point_counts       = cvCreateMat( n_boards, 1, CV_32SC1 );
    CvMat* intrinsic_matrix   = cvCreateMat( 3, 3, CV_32FC1 );
    CvMat* distortion_coeffs  = cvCreateMat( 5, 1, CV_32FC1 );
    CvMat* rotation_vector    = cvCreateMat(3,1, CV_32FC1);
    CvMat* translation_vector = cvCreateMat(3,1, CV_32FC1);
    CvMat* rotMat             = cvCreateMat(3,3,CV_32FC1);


    CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
    int corner_count;
    int successes = 0;
    int step, frame = 0;

    //IplImage *image = cvQueryFrame( capture );
    IplImage *image = cvCreateImage(cvSize(w,h), 8, 1);
    o= viam_oneshot(handle, bank, &image, NULL, 1);

    IplImage *gray_image = cvCreateImage( cvGetSize( image ), 8, 1 );

    // Capture Corner views loop until we've got n_boards
    // succesful captures (all corners on the board are found)

    while( successes < n_boards ){
        // Skp every board_dt frames to allow user to move chessboard
        if( frame++ % board_dt == 0 ){
            // Find chessboard corners:
//            int found = cvFindChessboardCorners( image, board_sz, corners,
// &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
            int found = cvFindChessboardCorners( image, board_sz, corners,
&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
            // Get subpixel accuracy on those corners
            //cvCvtColor( image, gray_image, CV_BGR2GRAY );
            gray_image = cvCloneImage(image);
            //cvShowImage( "Acquisition", image );
            cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ),
                cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            // Draw it
            cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
            cvShowImage( "Calibration", image );

            // If we got a good board, add it to our data
            if( corner_count == board_n ){
                step = successes*board_n;
                for( int i=step, j=0; j < board_n; ++i, ++j ){
                    CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
                    CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
                    CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
                    CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
                    CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;		  
                }
                CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
                successes++;
		//pattern view

		cout<<"successes: "<<successes<<endl;
            }
        }
        // Handle pause/unpause and ESC
        int c = cvWaitKey( 15 );
        if( c == 'p' ){
            c = 0;
            while( c != 'p' && c != 27 ){
                c = cvWaitKey( 250 );
            }
        }
        if( c == 27 )
            return 0;
        o= viam_oneshot(handle, bank, &image, NULL, 1);
        //image = cvQueryFrame( capture ); // Get next image
    } // End collection while loop

    // Allocate matrices according to how many chessboards found
    CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
    CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
    CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );
    CvMat* rotation_vector2 = cvCreateMat( successes, 3, CV_32FC1 );
    CvMat* translation_vector2 = cvCreateMat( successes, 3, CV_32FC1 );

    // Transfer the points into the correct size matrices
    for( int i = 0; i < successes*board_n; ++i ){
        CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
        CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
        CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
        CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
        CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
    }

    for( int i=0; i < successes; ++i ){
        CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
    }
    cvReleaseMat( &object_points );
    cvReleaseMat( &image_points );
    cvReleaseMat( &point_counts );

    // At this point we have all the chessboard corners we need
    // Initiliazie the intrinsic matrix such that the two focal lengths
    // have a ratio of 1.0

    CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
    CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

    // Calibrate the camera
/*
    cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
        intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );
*/
    cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
        intrinsic_matrix, distortion_coeffs, rotation_vector2, translation_vector2, CV_CALIB_FIX_ASPECT_RATIO );
    
    // Save the intrinsics and distortions
    cvSave( "Intrinsics.xml", intrinsic_matrix );
    cvSave( "Distortion.xml", distortion_coeffs );
    cvSave( "Rotation_vector.xml", rotation_vector2 );
    cvSave( "Translation_vector.xml", translation_vector2 );

    // Example of loading these matrices back in
    CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
    CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );

    // Build the undistort map that we will use for all subsequent frames
    IplImage* mapx = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 );
    IplImage* mapy = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 );
    IplImage *imgRes = cvCreateImage(cvSize(640,480), 8, 1);
    cvInitUndistortMap( intrinsic, distortion, mapx, mapy);
    
    // Find Extrinsic Camera Params
    //cvFindExtrinsicCameraParams2(object_points2, image_points2, intrinsic, distortion, rotation_vector, translation_vector );
    
    // Save the Rotation and Translation vector
    //cvSave( "Rotation_vector.xml", rotation_vector );
    //cvSave( "Translation_vector.xml", translation_vector );
    
     //transform the rotation vector in a rotation matrix
     //cvRodrigues2(rotation_vector,rotMat,NULL);
     //cvSave( "Rotation_matrix.xml", rotMat );

    // Run the camera to the screen, now showing the raw and undistorted image
    cvNamedWindow( "Undistort" );

    while( image ){
        IplImage *t = cvCloneImage( image );
        cvShowImage( "Calibration", image ); // Show raw image
        cvRemap( t, imgRes, mapx, mapy, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,  cvScalarAll(0) ); // undistort image
        cvReleaseImage( &t );
        cvShowImage( "Undistort", imgRes ); // Show corrected image

        // Handle pause/unpause and esc
        int c = cvWaitKey( 15 );
        if( c == 'p' ){
            c = 0;
            while( c != 'p' && c != 27 ){
                c = cvWaitKey( 250 );
            }
        }
        if( c == 27 )
            break;
        o= viam_oneshot(handle, bank, &image, NULL, 1);
        //image = cvQueryFrame( capture );
    }

    return 0;
}
