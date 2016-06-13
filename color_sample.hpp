#ifndef COLOR_SAMPLE_VERSION
#define COLOR_SAMPLE_VERSION 2013061202

// ------------------------------------------
//		Includes
// ------------------------------------------

#include <vector>
#include <sstream>


// ------------------------------------------
//		Defines
// ------------------------------------------


// ------------------------------------------
//		Structs
// ------------------------------------------



// ------------------------------------------
//		Classes
// ------------------------------------------

template <class DataT = int>
class ColorSample {
protected:
	//// Protected Structs ////
	struct CsRgb {
		DataT r, g, b;
		CsRgb() {}
		CsRgb(DataT _r, DataT _g, DataT _b) { r = _r; g = _g; b = _b; }
		DataT getColor(int color) { return color % 3 == 0 ? r : (color % 3 == 1 ? g : b); }
	};

	//// Protected Instances ////
	std::vector<CsRgb> colors_;


	//// Protected Methods ////
	void init_();

public:
	ColorSample() { init_(); }

	// Getter
	inline DataT getR(int number) { return colors_[number % colors_.size()].r; }
	inline DataT getG(int number) { return colors_[number % colors_.size()].g; }
	inline DataT getB(int number) { return colors_[number % colors_.size()].b; }
	inline DataT getColor(int number, int color) { return colors_[number % colors_.size()].getColor(color); }
	std::string getColorInfo(int number);

};

// ------------------------------------------
//		Methods
// ------------------------------------------

template <class DataT>
void ColorSample<DataT>::init_() {
	/* Designate color samples here */
	short c[][3] = {

		{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255},
		{0, 255, 255}, {255, 128, 0}, {255, 0, 128}, {0, 255, 128}, {128, 0, 255},
		{128, 255, 0}, {0, 128, 255}, {255, 128, 128}, {128, 255, 128}, {128, 128, 255},
		{128, 0, 0}, {0, 128, 0}, {0, 0, 128}, {128, 128, 0}, {128, 0, 128},
		{0, 128, 128}

	};

	// (1) Copy above arrays into vector
	for(int i = 0; i < sizeof(c) / sizeof(c[0][0]) / 3; i++)
		colors_.push_back(CsRgb(c[i][0], c[i][1], c[i][2]));
}

template <class DataT>
std::string ColorSample<DataT>::getColorInfo(int number) {
	std::stringstream ss;
	ss << "(" << getR(number) << ", " << getG(number) << ", " << getB(number) << ")";
	return ss.str();
}


#endif