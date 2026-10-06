# AR Repères — WebAR sur iPhone par recalage sur cible QR

Webapp (Safari iPhone, sans installation) : on vise une cible imprimée (QR + cadre noir) placée à une
position connue, puis on se déplace dans la pièce ; les objets connus (position, label, info) s'affichent
en surimpression sur la vidéo.

## Principe

```
cible A4 ──(zxing : identité + coins grossiers)──► homographie H0
         ──(affinage sous-pixel des 8 bords du cadre)──► 8 coins précis
         ──(PnP planaire + Levenberg-Marquardt)──► T_cam←cible   (métrique)
SLAM 8th Wall (scale: 'absolute') ──────────────► T_monde←cam     (chaque frame)
paires (repère pièce ↔ monde SLAM) accumulées sur 20–80 vues ──(Horn/Kabsch)──► T_monde←pièce
objet P_pièce ──► T_monde←pièce ──► T_cam←monde ──► projection ──► étiquette
```

- **Pourquoi un cadre noir ?** Les coins d'un QR donnés par les détecteurs sont à ±1 px (le coin
  bas-droit est extrapolé). Le cadre donne 8 droites ajustées sur ~24 points de gradient chacune,
  donc des coins à ~0,1–0,2 px. En simulation (cible A4, 0,5–1 m, 1280 px), l'erreur d'une vue sur un
  point à ~8 m passe de ~7 cm (coins QR seuls) à ~0,7 cm (cadre).
- **Pourquoi plusieurs vues ?** Le PnP sur une petite cible a une erreur angulaire qui se propage avec
  la distance. Accumuler des vues sous différents angles moyenne cette erreur.
- **Ce qui limite en pratique :** la dérive du SLAM web (typiquement 1–2 % de la distance parcourue),
  l'erreur d'échelle du mode `absolute`, et la planéité/le positionnement de la feuille. Compter
  quelques centimètres à 5–10 m. Une seconde cible au fond de la pièce (ancre `A2`) améliore
  fortement la rotation, car le recalage est calculé sur toutes les ancres vues.

## Fichiers

| Fichier | Rôle |
|---|---|
| `index.html` | page de l'app (charge 8th Wall et zxing-wasm depuis jsDelivr) |
| `app.js` | pipeline caméra, détection, recalage, affichage |
| `geom.js` | PnP planaire (DLT + LM), recalage rigide (Horn), utilitaires |
| `refine.js` | affinage sous-pixel des bords du cadre |
| `config.json` | ancres et objets |
| `cible.html` / `cible-A1.pdf` | cible à imprimer (QR 120 mm, cadre 190/170 mm) |

## Mise en route

1. **Imprimer** `cible-A1.pdf` à l'échelle 100 % (pas « ajuster à la page »). Mesurer le côté extérieur
   du cadre ; s'il ne fait pas 190,0 mm, reporter les vraies cotes dans `config.json` (en mètres :
   `frameOuter`, `frameInner`, `qrSide`). Coller la feuille bien à plat (idéalement sur carton plume).
   Pour d'autres ancres : ouvrir `cible.html`, changer l'identifiant, imprimer.
2. **Héberger en HTTPS** (obligatoire pour la caméra) : GitHub Pages, Netlify, ou pour tester depuis
   le Mac : `npx serve .` puis un tunnel HTTPS (`cloudflared tunnel --url http://localhost:3000`).
3. **Ouvrir sur l'iPhone dans Safari**, « Démarrer », autoriser caméra et mouvement.
4. Bouger un peu le téléphone (initialisation du SLAM et de l'échelle), viser la cible à 40–80 cm,
   **tourner lentement autour** (±30°) jusqu'à « Recalé ». Le contour devient vert quand le cadre est
   affiné (jaune = coins QR seuls, vues ignorées par défaut).
5. Se déplacer : les objets dans le champ ont une pastille + label + distance ; les objets hors champ
   sont signalés par une flèche au bord de l'écran ; l'objet visé (au centre, < 8°) ou touché affiche
   sa fiche en bas.

## Repère et coordonnées

Le repère pièce est celui de l'ancre `A1` (si `position`/`rotation` restent à zéro) :
origine au **centre du cadre**, **x vers la droite**, **y vers le haut**, **z sort de la feuille**
(vers la pièce si la cible est au mur). Unités : mètres.

Exemple : cible collée au mur, centre à 1,50 m du sol → le sol est à `y = -1.5`, un objet à 3 m
devant le mur et 2 m à droite, à 1 m du sol : `[2.0, -0.5, 3.0]`.

Ancres supplémentaires : `position` (centre du cadre dans le repère pièce) et `rotation` en degrés
(Euler, ordre XYZ comme Three.js, R = Rx·Ry·Rz). Une cible sur le mur d'en face (10 m), tournée vers
la première : `"position": [0, 0, 10], "rotation": [0, 180, 0]`.

**Relever des positions sur place (🎯 Relever)** : une fois recalé, choisir l'objet (ou « nouvel
objet »), viser le point avec la croix centrale et toucher « ◎ Viser » ; se déplacer latéralement
d'au moins 1 m et viser le même point ; idéalement une 3e visée. Le point est triangulé (moindres
carrés sur les rayons de visée), avec l'angle entre visées (viser ≥ 15°, idéalement 30–90°) et
l'écart des rayons. « Enregistrer » met l'objet à jour immédiatement dans l'app et copie la liste
`objects` au format JSON, à coller dans `config.json`.

## Diagnostic (bouton Debug)

- cercles cyan = coins mesurés dans l'image ; points magenta = coins reprojetés par la pose ;
  ils doivent coïncider (sinon : problème de correspondance pixels ↔ écran, voir `pixelRotation`).
- `reproj` = erreur de reprojection (px écran), `bords` = résidu des droites du cadre (px).
- `résidu` dans la barre d'état = écart RMS du recalage global (mm) ; s'il grimpe, « ↺ Recaler ».

Paramètres dans `config.json` (mêmes noms que `DEFAULTS` dans `app.js`) : `framesForLock`,
`maxFrames`, `maxReprojPx`, `requireFrame`, `focusAngleDeg`, `maxLabelDistance`,
`pixelMaxDimension`, `pixelRotation`.

## Limites connues

- Safari iOS n'expose pas WebXR : le suivi vient du SLAM du binaire 8th Wall (licence d'usage
  limitée, non open source, plus développé activement). Vérifier la licence avant une diffusion
  publique ; la mention « Powered by 8th Wall » est conservée.
- Pas de LiDAR ni d'occlusion en web. Distorsion de l'objectif non modélisée (faible sur iPhone).
- Hypothèse à valider au premier essai : le tableau de pixels caméra de 8th Wall couvre la même vue
  que le canevas (sinon recadrage « cover » supposé). Le mode Debug le vérifie immédiatement.
- Si le suivi est perdu longtemps (« Suivi limité »), le repère SLAM peut sauter : recaler.

## Test sans iPhone

`test/fake-xr8.js` remplace le moteur 8th Wall par un faux moteur qui rend des images synthétiques
de la cible sous des poses connues (et expose la vérité terrain dans `window.__truth`). Dans une copie
de `index.html`, remplacer le script 8th Wall par `qrcode.js` (qrcode-generator), `geom.js` puis
`test/fake-xr8.js`. Sur ce banc (sans dérive SLAM), le recalage atteint ~1 mm de résidu et quelques
millimètres d'erreur à 8–10 m : en conditions réelles, c'est le SLAM qui fixera la précision.
