Les différents modes implémentés sont les suivants :

frozen-direct : permet de tester computDKDetailed(), qui permet de voir les croix rouges sur chaque articulation

robot-ik : permet de tester computeIKOriented(), qui prend en paramètre les donnés targets dans le repère du robot liées au bout de la patte.

center-follow : Le centre de robot suit un point (x,y,z) déterminé par l'utilisateur. Le bout des pattes ne bouge pas.

control-legs : Avoir le contrôle sur chaque patte du robot (peut être placée à une position (x,y,z))

walk : Le robot marche en ligne droite, marche effectuée en 2 phases en bougeant les pieds 3 par 3. (utilise Oriented) La vitesse du robot est choisie par l'utilisateur.

walk-2by2 : la marche est effectuée en 3 phases en bougeant les pattes  2 par 2.

walk-advanced : walk + possibilité de contrôler la distance des pas effectués, l'orientation du robot

static_rotation : le robot effectue une rotation sans bouger ses pattes (restent collées au sol)

dynamic-rotation : le robot effectue une rotation sans bouger son centre (peut bouger ses pieds du sol). 

holonomic : la combinaison des mouvements précedents (walk-advanced + dynamic-rotation) en sommant leurs alphas. Ne fonctionne pas correctement. 
