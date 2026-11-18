import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../node_modules/three/build/three.module.js';
import {createRescueLayer} from '../rescue-view.js';
import {validPersonColor, mappingOptions, connectMappingSettings} from '../mapping-settings.js';

const person = {id: 1, center: [0, 0, 2], bounds: [[-.2,-.8,1.8],[.2,.8,2.2]],
  voxel_size: .06, voxels: [[0,0,2],[0,.06,2]], observed: true};

test('person voxels and their 3D outline share red, and per-vertex scene color cannot override them', () => {
  const layer = createRescueLayer(THREE);
  layer.update({person_color: '#ff3030', people: [person]});
  const objects = []; layer.group.traverse(object => objects.push(object));
  const voxels = objects.find(object => object.isInstancedMesh);
  assert.equal(voxels.count, 2);
  assert.equal(voxels.material.color.getHexString(), 'ff3030');
  assert.equal(objects.find(object => object.userData.personOutline).material.color.getHexString(), 'ff3030');
  layer.dispose();
});

test('saved person color and outline visibility survive replay, and last-seen markers remain visible', () => {
  const layer = createRescueLayer(THREE);
  layer.update({person_color: '#0088ff', person_outline: false, people: [{...person, voxels: [], observed: false}]});
  const objects = []; layer.group.traverse(object => objects.push(object));
  assert.equal(objects.filter(object => object.isInstancedMesh).length, 0);
  assert.equal(objects.find(object => object.userData.personOutline).visible, false);
  assert.ok(objects.some(object => object.isMesh && object.material.color.getHexString() === '0088ff'));
  layer.update(null); assert.equal(layer.group.visible, false); layer.dispose();
});

test('legacy saved meshes remain readable while new point maps create no static mesh', () => {
  const layer = createRescueLayer(THREE);
  layer.update({mesh:{vertices:[[0,0,2],[1,0,2],[0,1,2]],triangles:[[0,1,2]]},people:[]});
  assert.equal(layer.surfaceVisible, true);
  layer.update({render_mode:'points',point_revision:1,people:[person]});
  assert.equal(layer.surfaceVisible, false);
  layer.dispose();
});

test('person appearance changes immediately without another frame and persists for future mapping', () => {
  const elements = new Map();
  const original = globalThis.document;
  globalThis.document = {getElementById(id) {
    if (!elements.has(id)) elements.set(id, {dataset: {}, addEventListener(name, fn) { this[name] = fn; }});
    return elements.get(id);
  }};
  connectMappingSettings(); globalThis.document = original;
  const layer = createRescueLayer(THREE); layer.update({people: [person]});
  const input = elements.get('rescue-person-color'); input.value = '#00ff00'; input.input();
  let voxels; layer.group.traverse(object => { if (object.isInstancedMesh) voxels = object; });
  assert.equal(voxels.material.color.getHexString(), '00ff00');
  assert.equal(mappingOptions().person_color, '#00ff00');
  assert.equal(validPersonColor('red'), false); layer.dispose();
});
