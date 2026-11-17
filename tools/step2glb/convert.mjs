// Convierte "ENSAMBLAJE TARANTULA.step" en web/models/tarantula.glb
//
// El GLB contiene 4 nodos, cada uno con su geometría ya expresada en el
// sistema local de su articulación (metros, Z arriba, X radial hacia fuera):
//   body  -> sistema del cuerpo (origen en el centro, a la altura del eje de cadera)
//   coxa  -> origen en el eje de guiñada (q1), pose q1 = 0
//   femur -> origen en el eje de cadera (q2), pose q2 = 0
//   tibia -> origen en el eje de rodilla (q3), pose q3 = 0 (tibia alineada con el fémur)
// El visor monta la cadena cinemática con R_HIP, L1, L2 de Leg.h.
//
// Uso:  node convert.mjs "../../ENSAMBLAJE TARANTULA.step" ../../web/models/tarantula.glb

import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { Document, NodeIO } from '@gltf-transform/core';
import { MeshoptSimplifier } from 'meshoptimizer';

const require = createRequire(import.meta.url);
const occtimportjs = require('occt-import-js');

const [, , stepPath, outPath] = process.argv;
if (!stepPath || !outPath) {
  console.error('Uso: node convert.mjs <entrada.step> <salida.glb>');
  process.exit(1);
}

// ---------------------------------------------------------------------------
// Clasificación de las 29 piezas de cada pata (el orden es idéntico en las 4).
// Se usa la primera pata del STEP (la que apunta hacia -Y en el CAD).
// ---------------------------------------------------------------------------
const LINK_OF_PART = {
  0: 'femur',  // motor 6010 rodilla
  1: 'femur',  // tapa V15 rodilla
  2: 'coxa',   // motor 6010 cadera
  3: 'coxa',   // tapa V15 cadera
  4: 'femur',  // rodamiento rodilla
  5: 'coxa',   // motor 6010 coxa (eje vertical)
  6: 'coxa',   // tapa V15 coxa
  7: 'coxa',   // rodamiento cadera
  8: 'coxa',   // rodamiento coxa
  9: 'tibia',  // mini X
  10: 'femur', // maxi X
  11: 'coxa', 12: 'coxa', 13: 'coxa', 14: 'coxa', 15: 'coxa', 16: 'coxa', 17: 'coxa', // soporte coxa
  18: 'tibia', // D - Eslabon
  19: 'tibia', // Componente2
  20: 'femur', // Componente2 (2)
  21: 'tibia', // Componente3 (pie)
  22: 'femur', // Componente2 (3)
  23: 'femur', // separador rodilla
  24: 'femur', // separador cadera
  25: 'coxa',  // Soporte Rodamiento B
  26: 'coxa',  // Soporte Rodamiento C
  27: 'tibia', // brida de salida del motor de rodilla
  28: 'femur', // placa motor rodilla
};

const ALPHA_BODY = Math.PI / 4;  // giro del cuerpo CAD -> sistema del código
const SIMPLIFY_MIN_TRIS = 4000;  // piezas por encima de esto se simplifican
const SIMPLIFY_RATIO = 0.12;

const DEFAULT_COLOR = [0.62, 0.64, 0.68];

// ---------------------------------------------------------------------------
const occt = await occtimportjs();
const res = occt.ReadStepFile(new Uint8Array(fs.readFileSync(stepPath)), {
  linearUnit: 'millimeter',
  linearDeflectionType: 'bounding_box_ratio',
  linearDeflection: 0.0008,
  angularDeflection: 0.4,
});
if (!res.success) throw new Error('No se pudo leer el STEP');

const top = res.root.children[0];
const legNodes = top.children.filter(n => n.name.startsWith('PATA'));
const bodyNodes = top.children.filter(n => !n.name.startsWith('PATA'));
const legA = legNodes[0];
const meshOf = i => res.meshes[i];

function bboxCenter(m) {
  const p = m.attributes.position.array;
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3)
    for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[i + k]); mx[k] = Math.max(mx[k], p[i + k]); }
  return mn.map((v, k) => (v + mx[k]) / 2);
}

// Centro del cuerpo (placa superior)
const bodyC = bboxCenter(meshOf(bodyNodes[0].meshes[0]));
// Pivotes de la pata A, sacados del centro de cada motor
const coxaC = bboxCenter(meshOf(legA.meshes[5]));
const hipC = bboxCenter(meshOf(legA.meshes[2]));
const kneeC = bboxCenter(meshOf(legA.meshes[0]));
const zHip = hipC[2];

// Pata A: X_pata = -Y_cad, Y_pata = +X_cad
const toLeg = (x, y, z) => [-(y - coxaC[1]), x - coxaC[0], z - zHip];
const hipLeg = toLeg(...hipC);
const kneeLeg = toLeg(...kneeC);
console.log('Medidas CAD (mm):',
  'R_HIP', (bodyC[1] - coxaC[1]).toFixed(1),
  'L1', hipLeg[0].toFixed(1),
  'L2', (kneeLeg[0] - hipLeg[0]).toFixed(1),
  'dz rodilla', (kneeLeg[2] - hipLeg[2]).toFixed(1));

// Transformaciones de vértice (mm CAD -> m local)
const xf = {
  body: (x, y, z) => {
    const dx = x - bodyC[0], dy = y - bodyC[1], dz = z - zHip;
    const c = Math.cos(ALPHA_BODY), s = Math.sin(ALPHA_BODY);
    return [(c * dx - s * dy) / 1000, (s * dx + c * dy) / 1000, dz / 1000];
  },
  coxa: (x, y, z) => toLeg(x, y, z).map(v => v / 1000),
  femur: (x, y, z) => { const p = toLeg(x, y, z); return [(p[0] - hipLeg[0]) / 1000, p[1] / 1000, (p[2] - hipLeg[2]) / 1000]; },
  // En el CAD la tibia cuelga vertical (q2 = 0, q3 = -90°). La llevamos a q3 = 0:
  // p_tib = Ry(q3_cad) * p   ->  con q3_cad = -90°: x' = -z, z' = x
  tibia: (x, y, z) => {
    const p = toLeg(x, y, z);
    const px = p[0] - kneeLeg[0], py = p[1], pz = p[2] - kneeLeg[2];
    return [-pz / 1000, py / 1000, px / 1000];
  },
};
// Normales: misma rotación sin traslación
const nxf = {
  body: (x, y, z) => { const c = Math.cos(ALPHA_BODY), s = Math.sin(ALPHA_BODY); return [c * x - s * y, s * x + c * y, z]; },
  coxa: (x, y, z) => [-y, x, z],
  femur: (x, y, z) => [-y, x, z],
  tibia: (x, y, z) => { const p = [-y, x, z]; return [-p[2], p[1], p[0]]; },
};

await MeshoptSimplifier.ready;

function simplify(m) {
  const pos = new Float32Array(m.attributes.position.array);
  const idx = new Uint32Array(m.index.array);
  const tris = idx.length / 3;
  if (tris < SIMPLIFY_MIN_TRIS) return idx;
  const target = Math.floor(tris * SIMPLIFY_RATIO) * 3;
  const [out] = MeshoptSimplifier.simplify(idx, pos, 3, target, 0.01, ['LockBorder']);
  return out;
}

// Acumuladores por enlace y color
const groups = {}; // link -> colorKey -> {pos:[], nor:[], idx:[], color}
function add(link, m) {
  const color = m.color || DEFAULT_COLOR;
  const key = color.map(c => c.toFixed(3)).join(',');
  groups[link] ??= {};
  const g = (groups[link][key] ??= { pos: [], nor: [], idx: [], color });
  const P = m.attributes.position.array;
  const N = m.attributes.normal?.array;
  const idx = simplify(m);
  // compactar vértices usados
  const remap = new Map();
  const base = g.pos.length / 3;
  for (const i of idx) {
    if (!remap.has(i)) {
      remap.set(i, base + remap.size);
      g.pos.push(...xf[link](P[3 * i], P[3 * i + 1], P[3 * i + 2]));
      g.nor.push(...(N ? nxf[link](N[3 * i], N[3 * i + 1], N[3 * i + 2]) : [0, 0, 1]));
    }
    g.idx.push(remap.get(i));
  }
}

legA.meshes.forEach((mi, k) => add(LINK_OF_PART[k] ?? 'coxa', meshOf(mi)));
add('body', meshOf(bodyNodes[0].meshes[0]));
for (const n of bodyNodes.slice(1)) n.meshes.forEach(mi => add('body', meshOf(mi)));

// ---------------------------------------------------------------------------
const doc = new Document();
const buffer = doc.createBuffer();
const scene = doc.createScene('tarantula');
const materials = new Map();
function material(color) {
  const key = color.join(',');
  if (!materials.has(key)) {
    const lum = (color[0] + color[1] + color[2]) / 3;
    materials.set(key, doc.createMaterial(`m${materials.size}`)
      .setBaseColorFactor([...color, 1])
      .setMetallicFactor(lum > 0.3 && lum < 0.8 ? 0.6 : 0.15)
      .setRoughnessFactor(lum < 0.1 ? 0.55 : 0.4));
  }
  return materials.get(key);
}

let totalTris = 0;
for (const link of ['body', 'coxa', 'femur', 'tibia']) {
  const mesh = doc.createMesh(link);
  for (const g of Object.values(groups[link])) {
    const prim = doc.createPrimitive()
      .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(new Float32Array(g.pos)).setBuffer(buffer))
      .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(new Float32Array(g.nor)).setBuffer(buffer))
      .setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(g.idx)).setBuffer(buffer))
      .setMaterial(material(g.color));
    mesh.addPrimitive(prim);
    totalTris += g.idx.length / 3;
  }
  scene.addChild(doc.createNode(link).setMesh(mesh));
}

fs.mkdirSync(path.dirname(outPath), { recursive: true });
await new NodeIO().write(outPath, doc);
console.log(`OK -> ${outPath}  (${totalTris} triángulos, ${(fs.statSync(outPath).size / 1e6).toFixed(2)} MB)`);
