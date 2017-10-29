cc=g++ -std=c++11  #support C++11 standard
cvIpath=-I/usr/local/OpenCV/include/ #opencv include path
cvLib=`pkg-config --libs opencv`  # opencv libs


program =  ImageConsProd.o masters.o
ImageProd = ImageConsProd.o AngleSolver.o AngleSolverFactory.o\
			ArmorDetector.o Predictor.o ResFilter.o RMVideoCapture.o \
			RuneDetector.o Serial.o Setting.o

main: masters.o image.o
	cc -o masters masters.o image.o
masters : $program
	cc -c masters.cpp
Set.o : Setting.cpp Setting.hpp ParametersSet.hpp
	cc -c Setting.cpp $(cvIpath) $(cvLib)
Ser : Serial.hpp Serial.cpp
	cc -c Serial.cpp
RMV : RMVideoCapture.hpp RMVideoCapture.cpp
	cc -c RMVideoCapture.cpp
Res : ResFilter.cpp ResFilter.hpp
	cc -c ResFilter.cpp
Pre : Predictor.cpp Predictor.hpp
	cc -c Predictor.cpp
Arm : ArmorDetector.hpp ArmorDetector.cpp
	cc -c ArmorDetector.cpp
AngSF : AngleSolverFactory.hpp AngleSolverFactory.cpp
	cc -c AngleSolverFactory.cpp
AngS : AngleSolver.hpp AngleSolver.cpp
	cc -c AngleSolver.cpp
ImgP : ImageConsProd.cpp ImageConsProd.hpp 
	cc -c ImageConsProd.cpp
ImageProdLink :  $(ImageProd)
	cc -o image.o $(ImageProd)

clean:
	rm *.o masters