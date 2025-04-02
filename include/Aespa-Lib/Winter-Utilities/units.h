#pragma once


namespace aespa_lib {
namespace units {


// ---------- Length ----------

class Length {
public:
	Length(double value_tiles);

	Length operator-();
	Length operator+(Length other);
	Length operator-(Length other);
	Length &operator+=(Length other);
	Length &operator-=(Length other);

	double tiles();
	double in();
	double qtIn();
	double cm();
	double m();

private:
	double value_tiles;
};


// ---------- Angle ----------

class PolarAngle {
public:
	PolarAngle(double value_polarDeg);

	PolarAngle operator-();
	PolarAngle operator+(PolarAngle other);
	PolarAngle operator-(PolarAngle other);
	PolarAngle &operator+=(PolarAngle other);
	PolarAngle &operator-=(PolarAngle other);

	double polarDeg();
	double polarRad();

private:
	double value_polarDeg;
};


// ---------- Literals ----------

inline namespace literals {


inline namespace length {

Length operator ""_tiles(long double value);
Length operator ""_in(long double value);
Length operator ""_qtIn(long double value);
Length operator ""_cm(long double value);
Length operator ""_m(long double value);

Length operator ""_tiles(unsigned long long value);
Length operator ""_in(unsigned long long value);
Length operator ""_qtIn(unsigned long long value);
Length operator ""_cm(unsigned long long value);
Length operator ""_m(unsigned long long value);

}


inline namespace angle {

PolarAngle operator ""_polarDeg(long double value);
PolarAngle operator ""_polarRad(long double value);

PolarAngle operator ""_polarDeg(unsigned long long value);
PolarAngle operator ""_polarRad(unsigned long long value);

}


}

}
}
