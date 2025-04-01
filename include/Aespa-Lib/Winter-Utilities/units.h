#pragma once


namespace aespa_lib {
namespace units {


// ---------- Length ----------

class Length {
public:
	Length(double tiles);

	Length operator-();

	double tiles();
	double in();
	double qtIn();
	double cm();
	double m();

private:
	double value_tiles;
};


// ---------- Literals ----------

inline namespace literals {

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

}
}
