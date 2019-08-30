#LV5.cpp

Trodimenzionalna rekonstrukcija scene iz svije slike

Pokretanjem programa pojavljuju se dva prozora "Left image" i "Right image", koji prikazuju isti objekt uslikan iz dva različita kuta. Potom se pojavljuju prozori s detektiranim SIFT značajkama na lijevoj i desnoj slici. Nakon njih pojavljuje se prozor "Keypoint matches", koji prikazuje sparene značajke na slikama, pomoću lokalnih deskriptora. Nakon njega slijedi prozor "Keypoint matches filtered", koji pokazuje filtrirane sparene značajke, pomoću fundamentalne matrice. Na kraju se detektirane 3D točke zapisuju u datoteku "points3d.txt". Ta se datoteka učita u matlab, iz skripte "plot3DPoints.m", koja vizualizira detektirane 3D točke.
