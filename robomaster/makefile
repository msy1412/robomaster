gc=g++ -std=c++11 -g

#support C++11 standard
cvIpath=-I/usr/local/OpenCV/include/ #opencv include path
cvLib=-L/usr/local/OpenCV/lib -lopencv_shape -lopencv_stitching\
 	-lopencv_objdetect -lopencv_superres -lopencv_videostab \
	 -lopencv_calib3d -lopencv_features2d -lopencv_highgui \
	 -lopencv_videoio -lopencv_imgcodecs -lopencv_video \
	 -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann \
	 -lopencv_core  
	 # opencv libs


program =  ImageConsProd.o masters.o
ImageProd = RMVideoCapture.o Serial.o Settings.o AngleSolver.o \
			AngleSolverFactory.o\
			ArmorDetector.o Predictor.o ResFilter.o  \
			RuneDetector.o  ImageConsProd.o

main : masters ImageProdLink
	g++ -o masters masters.o $(ImageProd) $(cvIpath) ${cvLib} -lpthread


masters : masters.cpp
	${gc} -c masters.cpp
ImageProdLink :  
	${gc} -c Settings.cpp 
	${gc} -c Serial.cpp 
	${gc} -c RMVideoCapture.cpp 
	${gc} -c ResFilter.cpp 
	${gc} -c Predictor.cpp 
	${gc} -c ArmorDetector.cpp 
	${gc} -c AngleSolverFactory.cpp 
	${gc} -c AngleSolver.cpp	
	${gc} -c RuneDetector.cpp
	${gc} -c ImageConsProd.cpp

clean:
	rm *.o masters

all:
	echo ${cvLib}