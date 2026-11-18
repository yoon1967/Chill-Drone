import {mappingSettings, onMappingSettings} from './mapping-settings.js';
const palette = [0x899b9c, 0xff3030, 0xa8bac4, 0x7f9a80, 0xe2dcc2, 0xc29462, 0x7aacc1, 0xb58aab, 0x80c9d2];
export function createRescueLayer(THREE) {
  const group = new THREE.Group(); group.name = 'rescue-map';
  const surfaces = new THREE.Group(), people = new THREE.Group(); group.add(surfaces, people);
  const cube = new THREE.BoxGeometry(1, 1, 1);
  const personMaterial = new THREE.MeshStandardMaterial({color: '#ff3030', emissive: '#ff3030', emissiveIntensity: .25, roughness: .8});
  const outlineMaterial = new THREE.LineBasicMaterial({color: '#ff3030'});
  const lastSeenMaterial = new THREE.LineDashedMaterial({color: '#ff3030', dashSize: .06, gapSize: .035, transparent: true, opacity: .65});
  let value = null, meshKey = null;
  function clear(target) {
    target.traverse(child => {
      if (child.geometry && child.geometry !== cube) child.geometry.dispose();
      if (child.isInstancedMesh) child.dispose();
      if (child.isSprite) { child.material.map?.dispose(); child.material.dispose(); }
      if (child.userData.surface) child.material.dispose();
    }); target.clear();
  }
  function appearance() {
    const prefs = mappingSettings();
    const color = prefs.appearance_changed ? prefs.person_color : value?.person_color || prefs.person_color;
    personMaterial.color.set(color); personMaterial.emissive.set(color); outlineMaterial.color.set(color); lastSeenMaterial.color.set(color);
    const outline = prefs.appearance_changed ? prefs.person_outline : value?.person_outline !== false;
    people.traverse(child => { if (child.userData.personOutline) child.visible = outline; });
    for (const mesh of surfaces.children) {
      const source = mesh.userData.source, colors = new Float32Array(source.vertices.length * 3);
      source.vertices.forEach((_, i) => {
        const c = new THREE.Color(palette[source.labels?.[i] || 0]);
        colors.set(prefs.semantic_colors ? [c.r, c.g, c.b] : source.colors?.[i] || [.55, .65, .67], i * 3);
      }); mesh.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }
  }
  const unsubscribe = onMappingSettings(appearance);
  function addMesh(source) {
    if (!source?.vertices?.length || !source.triangles?.length) return;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(source.vertices.flat(), 3)); geometry.setIndex(source.triangles.flat());
    if (source.normals?.length === source.vertices.length) geometry.setAttribute('normal', new THREE.Float32BufferAttribute(source.normals.flat(), 3));
    else geometry.computeVertexNormals(); geometry.computeBoundingSphere();
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({vertexColors: true, side: THREE.DoubleSide, roughness: .9}));
    mesh.userData = {surface: true, source}; surfaces.add(mesh);
  }
  function update(data) {
    value = data || null; group.visible = Boolean(data);
    if (!data) { clear(surfaces); clear(people); meshKey = null; return; }
    const nextKey = data.mesh_revision == null ? data : `${data.map_id}:${data.chunk}:${data.mesh_revision}`;
    if (meshKey !== nextKey) { clear(surfaces); for (const mesh of data.meshes || [data.mesh]) addMesh(mesh); meshKey = nextKey; }
    clear(people); const matrix = new THREE.Matrix4();
    for (const person of data.people || []) {
      const size = person.voxel_size; if (!(size > 0) || !person.bounds?.length) continue;
      if (person.voxels?.length) {
        const voxels = new THREE.InstancedMesh(cube, personMaterial, person.voxels.length);
        person.voxels.forEach((position, index) => { matrix.makeScale(size * .92, size * .92, size * .92); matrix.setPosition(...position); voxels.setMatrixAt(index, matrix); });
        voxels.instanceMatrix.needsUpdate = true; voxels.computeBoundingSphere(); voxels.userData.personId = person.id; people.add(voxels);
      } else {
        const marker = new THREE.Mesh(cube, personMaterial); marker.position.fromArray(person.center); marker.scale.setScalar(size * 1.2); people.add(marker);
      }
      const lo = new THREE.Vector3(...person.bounds[0]), hi = new THREE.Vector3(...person.bounds[1]);
      const dimensions = hi.clone().sub(lo).max(new THREE.Vector3(size, size, size));
      const box = new THREE.BoxGeometry(...dimensions.toArray());
      const outline = new THREE.LineSegments(new THREE.EdgesGeometry(box), person.observed ? outlineMaterial : lastSeenMaterial);
      box.dispose(); outline.position.copy(lo.add(hi).multiplyScalar(.5)); outline.userData.personOutline = true; outline.computeLineDistances(); people.add(outline);
      if (typeof document !== 'undefined') {
        const canvas = document.createElement('canvas'); canvas.width = 420; canvas.height = 64; const context = canvas.getContext('2d');
        context.fillStyle = 'rgba(12,20,25,.85)'; context.fillRect(0, 0, 420, 64); context.fillStyle = '#ffffff'; context.font = '24px sans-serif';
        context.fillText(`사람 ${person.id} · ${person.observed ? '현재 관측' : `마지막 ${Math.round(person.age_s || 0)}초 전`}`, 12, 41);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(canvas), depthTest: true}));
        const above = data.display_y_up ? person.bounds[1][1] + size * 2 : person.bounds[0][1] - size * 2;
        sprite.position.set(person.center[0], above, person.center[2]); sprite.scale.set(size * 10, size * 1.5, 1); people.add(sprite);
      }
    } appearance();
  }
  return {group, update, get surfaceVisible() { return surfaces.children.length > 0; }, dispose() { unsubscribe(); clear(surfaces); clear(people); cube.dispose();
    personMaterial.dispose(); outlineMaterial.dispose(); lastSeenMaterial.dispose(); group.removeFromParent(); }};
}
