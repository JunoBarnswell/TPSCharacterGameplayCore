// World coordinates are pixels; Y grows downwards. Every level is authored as data.
const ground = (start, end) => ({ x: start, y: 500, w: end - start, h: 60, kind: "ground" });
const ledge = (x, y, w) => ({ x, y, w, h: 22, kind: "ledge" });
const coin = (x, y) => ({ x, y });
const enemy = (x, minX, maxX, y = 466, speed = 70) => ({ x, y, minX, maxX, speed });

export const LEVELS = Object.freeze([
  {
    id: "meadow",
    name: "风铃草原",
    width: 2200,
    sky: ["#89dce4", "#e8f7de"],
    spawn: { x: 76, y: 462 },
    solids: [
      ground(0, 650), ground(760, 1390), ground(1500, 2200),
      ledge(265, 405, 150), ledge(490, 345, 130), ledge(865, 402, 155),
      ledge(1160, 360, 130), ledge(1570, 390, 155), ledge(1810, 340, 135),
    ],
    coins: [coin(330, 372), coin(555, 310), coin(685, 395), coin(895, 370),
      coin(958, 370), coin(1225, 325), coin(1450, 400), coin(1625, 355),
      coin(1870, 305), coin(2025, 450)],
    enemies: [enemy(390, 310, 570), enemy(1000, 810, 1260), enemy(1725, 1550, 2050)],
    hazards: [],
    checkpoint: { x: 1080, y: 440 },
    finish: { x: 2090, y: 420 },
  },
  {
    id: "canyon",
    name: "赤岩峡谷",
    width: 2660,
    sky: ["#ffc59c", "#fae4c8"],
    spawn: { x: 80, y: 462 },
    solids: [
      ground(0, 550), ground(680, 1130), ground(1250, 1820), ground(1950, 2660),
      ledge(345, 390, 120), ledge(730, 350, 120), ledge(875, 390, 145),
      ledge(1145, 335, 130), ledge(1430, 380, 140), ledge(1730, 335, 130),
      ledge(2025, 380, 170), ledge(2290, 325, 135),
    ],
    coins: [coin(395, 355), coin(650, 315), coin(795, 380), coin(895, 355),
      coin(1200, 300), coin(1355, 385), coin(1490, 345), coin(1790, 300),
      coin(2100, 345), coin(2355, 290), coin(2510, 448)],
    enemies: [enemy(310, 200, 520, 466, 85), enemy(920, 760, 1090, 466, 90),
      enemy(1500, 1320, 1780, 466, 105), enemy(2230, 2000, 2510, 466, 100)],
    hazards: [{ x: 1000, y: 478, w: 74, h: 22 }, { x: 1600, y: 478, w: 65, h: 22 }],
    checkpoint: { x: 1300, y: 440 },
    finish: { x: 2550, y: 420 },
  },
  {
    id: "night",
    name: "星灯高地",
    width: 3040,
    sky: ["#303e6d", "#8c8cbb"],
    spawn: { x: 80, y: 462 },
    solids: [
      ground(0, 590), ground(720, 1090), ground(1240, 1640),
      ground(1780, 2350), ground(2480, 3040),
      ledge(410, 395, 110), ledge(780, 345, 125), ledge(850, 390, 145),
      ledge(1290, 315, 110), ledge(1450, 385, 120), ledge(1695, 325, 125),
      ledge(2050, 380, 125), ledge(2310, 325, 135), ledge(2700, 365, 140),
    ],
    coins: [coin(470, 360), coin(700, 310), coin(800, 390), coin(1040, 335),
      coin(1210, 280), coin(1500, 350), coin(1750, 290), coin(1880, 445),
      coin(2100, 345), coin(2370, 290), coin(2570, 445), coin(2760, 330),
      coin(2900, 445)],
    enemies: [enemy(330, 170, 560, 466, 110), enemy(900, 750, 1060, 466, 115),
      enemy(1480, 1300, 1600, 466, 115), enemy(2020, 1830, 2280, 466, 125),
      enemy(2730, 2520, 2940, 466, 130)],
    hazards: [{ x: 810, y: 478, w: 65, h: 22 },
      { x: 1900, y: 478, w: 72, h: 22 }, { x: 2600, y: 478, w: 62, h: 22 }],
    checkpoint: { x: 1825, y: 440 },
    finish: { x: 2935, y: 420 },
  },
]);
