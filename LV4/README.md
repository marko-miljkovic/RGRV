#LV4.cpp

Prepoznavanje 2D objekata

Pokretanjem programa pojavljuju se dva prozora "Referent image" i "Working image", na kojima su prikazane slika iz koje ćemo izrezati objekt od interesa, te radna slika scene, na kojoj će se taj objekt tražiti. Potom se pojavljuje prozor "Cropping the image", gdje izrezujemo objekt od interesa (postupak izrezivanja jednak je kao u LV3). Nakon toga pojavljuje se prozor koji pokazuje izrezanu sliku. Potom slijede prozori koji prikazuju SIFT značajke detektirane na izrezanoj, te na radnoj slici. Idući prozor "Keypoint matches" prikazuje spojene značajke dviju slika, na temelju lokalnih deskriptora. Zadnji prozor "Detected object" prikazuje detektirani objekt od interesa na radnoj slici, unutar zelenog kvadrata.
