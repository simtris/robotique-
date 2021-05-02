Les différents modes implémentés sont les suivants :

frozen-direct : permet de tester computDKDetailed(), cette dernière permet de voir les croix rouge sur chaque articulation

robot-ik : permet de tester computeIKOriented(), cette dernière prend en paramettre les donnés targets dans le repère du robot liée au bout de la patte.

walk : Le robot marche en ligne droite, elle est effectué en 2 phase en bougeant les pieds 3 par 3. (utilise Oriented) La vitesse du robot est choisie par l'utilisateur.

walk-2by2 : la marche est effectué en 3 phase en bougeant les pieds 2 par 2.

walk-advanced : walk + possibilité de controller la distance des pas effectués, le sens d'orientation du robot,


center-follow : Le centre de robot suit un point (x,y,z) déterminé par l'utilisateur. Le bout des pattes ne bouge pas.

control-legs : Avoir le controlle sur chaque patte du robot (peut être placée à une position (x,y,z))

static_rotation : le robot effectue une rotation sans bouger ses pattes (restent collé au sol) (implémenté parcequ'on pensait que c'était ce qui est demandé mais c'est dynamic-rotation qui est demandé)

dynamic-rotation : le robot effectue une rotation sans bouger son centre (peut bouger ses pieds du sol). 

holonomic : la combinaison des mouvements précedent (walk-advanced + dynamic-rotation) en sommant leurs alphas. ne marche pas comme il faut.