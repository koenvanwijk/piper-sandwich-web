// Adapted from zalo/mujoco_wasm (ISC) — geometry builder + coordinate helpers only.
import * as THREE from 'three';
import { Reflector } from 'three/addons/objects/Reflector.js';

export async function loadSceneFromURL(mujoco, filename, parent) {
    // Free the old model and data (wasm heap objects are not garbage-collected).
    if (parent.data != null) {
      parent.data.delete();
      parent.data = null;
    }
    if (parent.model != null) {
      parent.model.delete();
      parent.model = null;
    }

    // Load in the state from XML.
    parent.model = mujoco.MjModel.mj_loadXML("/working/"+filename);
    parent.data  = new mujoco.MjData(parent.model);

    let model = parent.model;
    let data = parent.data;

    // Decode the null-terminated string names.
    let textDecoder = new TextDecoder("utf-8");
    let names_array = new Uint8Array(model.names);
    let fullString = textDecoder.decode(model.names);
    let names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

    // Create the root object.
    let mujocoRoot = new THREE.Group();
    mujocoRoot.name = "MuJoCo Root";
    parent.scene.add(mujocoRoot);

    /** @type {Object.<number, THREE.Group>} */
    let bodies = {};
    /** @type {Object.<number, THREE.BufferGeometry>} */
    let meshes = {};
    /** @type {THREE.Light[]} */
    let lights = [];

    // Default material definition.
    let material = new THREE.MeshPhysicalMaterial();
    material.color = new THREE.Color(1, 1, 1);
    
    // Loop through the MuJoCo geoms and recreate them in three.js.
    for (let g = 0; g < model.ngeom; g++) {
      // Only visualize geom groups up to 2 (same default behavior as simulate).
      if (!(model.geom_group[g] < 3)) { continue; }

      // Get the body ID and type of the geom.
      let b    = model.geom_bodyid[g];
      let type = model.geom_type  [g];
      let size = [
        model.geom_size[(g*3) + 0],
        model.geom_size[(g*3) + 1],
        model.geom_size[(g*3) + 2]
      ];

      // Create the body if it doesn't exist.
      if (!(b in bodies)) {
        bodies[b] = new THREE.Group();
        
        let start_idx = model.name_bodyadr[b];
        let end_idx = start_idx;
        while (end_idx < names_array.length && names_array[end_idx] !== 0) {
          end_idx++;
        }
        let name_buffer = names_array.subarray(start_idx, end_idx);
        bodies[b].name = textDecoder.decode(name_buffer);
        
        bodies[b].bodyID = b;
        bodies[b].has_custom_mesh = false;
      }

      // Set the default geometry. In MuJoCo, this is a sphere.
      let geometry = new THREE.SphereGeometry(size[0] * 0.5);
      if (type == mujoco.mjtGeom.mjGEOM_PLANE.value) {
        // Special handling for plane later.
      } else if (type == mujoco.mjtGeom.mjGEOM_HFIELD.value) {
        // TODO: Implement this.
      } else if (type == mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        geometry = new THREE.SphereGeometry(size[0]);
      } else if (type == mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
        geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
      } else if (type == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
        geometry = new THREE.SphereGeometry(1); // Stretch this below
      } else if (type == mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
        geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_BOX.value) {
        geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_MESH.value) {
        let meshID = model.geom_dataid[g];

        if (!(meshID in meshes)) {
          geometry = new THREE.BufferGeometry();

          let vertex_buffer = model.mesh_vert.subarray(
             model.mesh_vertadr[meshID] * 3,
            (model.mesh_vertadr[meshID]  + model.mesh_vertnum[meshID]) * 3);
          for (let v = 0; v < vertex_buffer.length; v+=3){
            //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
            let temp             =  vertex_buffer[v + 1];
            vertex_buffer[v + 1] =  vertex_buffer[v + 2];
            vertex_buffer[v + 2] = -temp;
          }

          let normal_buffer = model.mesh_normal.subarray(
             model.mesh_normaladr[meshID] * 3,
            (model.mesh_normaladr[meshID]  + model.mesh_normalnum[meshID]) * 3);
          for (let v = 0; v < normal_buffer.length; v+=3){
            //normal_buffer[v + 0] =  normal_buffer[v + 0];
            let temp             =  normal_buffer[v + 1];
            normal_buffer[v + 1] =  normal_buffer[v + 2];
            normal_buffer[v + 2] = -temp;
          }

          let uv_buffer = model.mesh_texcoord.subarray(
             model.mesh_texcoordadr[meshID] * 2,
            (model.mesh_texcoordadr[meshID]  + model.mesh_texcoordnum[meshID]) * 2);

          let face_to_vertex_buffer = model.mesh_face.subarray(
             model.mesh_faceadr[meshID] * 3,
            (model.mesh_faceadr[meshID]  + model.mesh_facenum[meshID]) * 3);
          let face_to_uv_buffer = model.mesh_facetexcoord.subarray(
             model.mesh_faceadr[meshID] * 3,
            (model.mesh_faceadr[meshID]  + model.mesh_facenum[meshID]) * 3);
          let face_to_normal_buffer = model.mesh_facenormal.subarray(
             model.mesh_faceadr[meshID] * 3,
            (model.mesh_faceadr[meshID]  + model.mesh_facenum[meshID]) * 3);

          // The UV and Normal Buffers are actually indexed by the triangle indices through the face_to_uv_buffer and face_to_normal_buffer.
          // We need to swizzle them into a per-vertex format for three.js
          let swizzled_uv_buffer      = new Float32Array((vertex_buffer.length / 3) * 2);
          let swizzled_normal_buffer  = new Float32Array(vertex_buffer.length);
          for (let t = 0; t < face_to_vertex_buffer.length / 3; t++) {
            let vi0 = face_to_vertex_buffer[(t * 3) + 0];
            let vi1 = face_to_vertex_buffer[(t * 3) + 1];
            let vi2 = face_to_vertex_buffer[(t * 3) + 2];
            let uvi0 = face_to_uv_buffer[(t * 3) + 0];
            let uvi1 = face_to_uv_buffer[(t * 3) + 1];
            let uvi2 = face_to_uv_buffer[(t * 3) + 2];
            let nvi0 = face_to_normal_buffer[(t * 3) + 0];
            let nvi1 = face_to_normal_buffer[(t * 3) + 1];
            let nvi2 = face_to_normal_buffer[(t * 3) + 2];
            swizzled_uv_buffer[(vi0 * 2) + 0] = uv_buffer[(uvi0 * 2) + 0];
            swizzled_uv_buffer[(vi0 * 2) + 1] = uv_buffer[(uvi0 * 2) + 1];
            swizzled_uv_buffer[(vi1 * 2) + 0] = uv_buffer[(uvi1 * 2) + 0];
            swizzled_uv_buffer[(vi1 * 2) + 1] = uv_buffer[(uvi1 * 2) + 1];
            swizzled_uv_buffer[(vi2 * 2) + 0] = uv_buffer[(uvi2 * 2) + 0];
            swizzled_uv_buffer[(vi2 * 2) + 1] = uv_buffer[(uvi2 * 2) + 1];
            swizzled_normal_buffer[(vi0 * 3) + 0] = normal_buffer[(nvi0 * 3) + 0];
            swizzled_normal_buffer[(vi0 * 3) + 1] = normal_buffer[(nvi0 * 3) + 1];
            swizzled_normal_buffer[(vi0 * 3) + 2] = normal_buffer[(nvi0 * 3) + 2];
            swizzled_normal_buffer[(vi1 * 3) + 0] = normal_buffer[(nvi1 * 3) + 0];
            swizzled_normal_buffer[(vi1 * 3) + 1] = normal_buffer[(nvi1 * 3) + 1];
            swizzled_normal_buffer[(vi1 * 3) + 2] = normal_buffer[(nvi1 * 3) + 2];
            swizzled_normal_buffer[(vi2 * 3) + 0] = normal_buffer[(nvi2 * 3) + 0];
            swizzled_normal_buffer[(vi2 * 3) + 1] = normal_buffer[(nvi2 * 3) + 1];
            swizzled_normal_buffer[(vi2 * 3) + 2] = normal_buffer[(nvi2 * 3) + 2];
          }
          // Copy out of the WASM heap: later heap growth (e.g. allocating MjData)
          // detaches these views and would turn the vertices into NaN.
          geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(vertex_buffer), 3));
          geometry.setAttribute("normal"  , new THREE.BufferAttribute(swizzled_normal_buffer, 3));
          geometry.setAttribute("uv"      , new THREE.BufferAttribute(swizzled_uv_buffer, 2));
          geometry.setIndex    (Array.from(face_to_vertex_buffer));
          geometry.computeVertexNormals(); // MuJoCo Normals acting strangely... just recompute them
          meshes[meshID] = geometry;
        } else {
          geometry = meshes[meshID];
        }

        bodies[b].has_custom_mesh = true;
      }
      // Done with geometry creation.

      // Set the Material Properties of incoming bodies
      let texture = undefined;
      let color = [
        model.geom_rgba[(g * 4) + 0],
        model.geom_rgba[(g * 4) + 1],
        model.geom_rgba[(g * 4) + 2],
        model.geom_rgba[(g * 4) + 3]];
      if (model.geom_matid[g] != -1) {
        let matId = model.geom_matid[g];
        color = [
          model.mat_rgba[(matId * 4) + 0],
          model.mat_rgba[(matId * 4) + 1],
          model.mat_rgba[(matId * 4) + 2],
          model.mat_rgba[(matId * 4) + 3]];

        // Construct Texture from model.tex_data
        texture = undefined;
        // mat_texid is now a matrix (nmat x mjNTEXROLE)
        // We use mjTEXROLE_RGB (value 1) for standard diffuse/color textures
        const mjNTEXROLE = 10; // Total number of texture roles
        const mjTEXROLE_RGB = 1; // RGB texture role
        let texId = model.mat_texid[(matId * mjNTEXROLE) + mjTEXROLE_RGB];
        
        if (texId != -1) {
          let width    = model.tex_width [texId];
          let height   = model.tex_height[texId];
          // tex_adr is an mjtSize (64-bit) field, exposed as a BigInt64Array.
          let offset   = Number(model.tex_adr[texId]);
          let channels = model.tex_nchannel[texId];
          let texData  = model.tex_data;
          let rgbaArray = new Uint8Array(width * height * 4);
          for (let p = 0; p < width * height; p++){
            rgbaArray[(p * 4) + 0] = texData[offset + ((p * channels) + 0)];
            rgbaArray[(p * 4) + 1] = channels > 1 ? texData[offset + ((p * channels) + 1)] : rgbaArray[(p * 4) + 0];
            rgbaArray[(p * 4) + 2] = channels > 2 ? texData[offset + ((p * channels) + 2)] : rgbaArray[(p * 4) + 0];
            rgbaArray[(p * 4) + 3] = channels > 3 ? texData[offset + ((p * channels) + 3)] : 255;
          }
          texture = new THREE.DataTexture(rgbaArray, width, height, THREE.RGBAFormat, THREE.UnsignedByteType);
          if (texId == 2) {
            texture.repeat = new THREE.Vector2(50, 50);
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
          } else {
            texture.repeat = new THREE.Vector2(model.mat_texrepeat[(model.geom_matid[g] * 2) + 0],
                                               model.mat_texrepeat[(model.geom_matid[g] * 2) + 1]);
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
          }

          texture.needsUpdate = true;
        }
      }

      // Create a new material for each geom to avoid cross-contamination
      let currentMaterial = new THREE.MeshPhysicalMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        transparent: color[3] < 1.0,
        opacity: color[3]/255.,
        specularIntensity: model.geom_matid[g] != -1 ?       model.mat_specular   [model.geom_matid[g]] : undefined,
        reflectivity     : model.geom_matid[g] != -1 ?       model.mat_reflectance[model.geom_matid[g]] : undefined,
        roughness        : model.geom_matid[g] != -1 ? 1.0 - model.mat_shininess  [model.geom_matid[g]] : undefined,
        metalness        : model.geom_matid[g] != -1 ?       0.1 : undefined, //model.mat_metallic   [model.geom_matid[g]]
        map              : texture
      });

      let mesh;// = new THREE.Mesh();
      if (type == 0) {
        mesh = new Reflector( new THREE.PlaneGeometry( 100, 100 ), { clipBias: 0.003, texture: texture } );
        mesh.rotateX( - Math.PI / 2 );
      } else {
        mesh = new THREE.Mesh(geometry, currentMaterial);
      }

      mesh.castShadow = g == 0 ? false : true;
      mesh.receiveShadow = type != 7;
      mesh.bodyID = b;
      bodies[b].add(mesh);
      getPosition  (model.geom_pos, g, mesh.position  );
      if (type != 0) { getQuaternion(model.geom_quat, g, mesh.quaternion); }
      if (type == 4) { mesh.scale.set(size[0], size[2], size[1]); } // Stretch the Ellipsoid
    }

    // Parse tendons.
    let tendonMat = new THREE.MeshPhongMaterial();
    tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);
    mujocoRoot.cylinders = new THREE.InstancedMesh(
        new THREE.CylinderGeometry(1, 1, 1),
        tendonMat, 1023);
    mujocoRoot.cylinders.receiveShadow = true;
    mujocoRoot.cylinders.castShadow    = true;
    mujocoRoot.add(mujocoRoot.cylinders);
    mujocoRoot.spheres = new THREE.InstancedMesh(
        new THREE.SphereGeometry(1, 10, 10),
        tendonMat, 1023);
    mujocoRoot.spheres.receiveShadow = true;
    mujocoRoot.spheres.castShadow    = true;
    mujocoRoot.add(mujocoRoot.spheres);

    // Parse lights.
    for (let l = 0; l < model.nlight; l++) {
      let light = new THREE.DirectionalLight();
      if (model.light_type[l] == 0) {
        light = new THREE.SpotLight();
        light.angle = 1.51;//model.light_cutoffangle[l];
      } else if (model.light_type[l] == 1) {
        light = new THREE.DirectionalLight();
      } else if (model.light_type[l] == 2) {
        light = new THREE.PointLight();
      }else if (model.light_type[l] == 3) {
        light = new THREE.HemisphereLight();
      }

      light.angle = 1.11;

      light.decay = model.light_attenuation[l] * 100;
      light.penumbra = 0.5;
      light.castShadow = true; // default false
      light.intensity = light.intensity * 3.14 * 1.0;

      light.shadow.mapSize.width = 1024; // default
      light.shadow.mapSize.height = 1024; // default
      light.shadow.camera.near = 0.1; // default
      light.shadow.camera.far = 10; // default
      //bodies[model.light_bodyid()].add(light);
      if (bodies[0]) {
        bodies[0].add(light);
      } else {
        mujocoRoot.add(light);
      }
      lights.push(light);
    }
    if (model.nlight == 0) {
      let light = new THREE.DirectionalLight();
      mujocoRoot.add(light);
    }

    for (let b = 0; b < model.nbody; b++) {
      //let parent_body = model.body_parentid()[b];
      if (b == 0 || !bodies[0]) {
        mujocoRoot.add(bodies[b]);
      } else if(bodies[b]){
        bodies[0].add(bodies[b]);
      } else {
        console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
        bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; bodies[b].bodyID = b; bodies[b].has_custom_mesh = false;
        bodies[0].add(bodies[b]);
      }
    }
  
    parent.mujocoRoot = mujocoRoot;

    return [model, data, bodies, lights];
}


export function drawTendonsAndFlex(mujocoRoot, model, data) {
  // Update tendon transforms.
  let identityQuat = new THREE.Quaternion();
  let numWraps = 0;
  if (mujocoRoot && mujocoRoot.cylinders) {
    let mat = new THREE.Matrix4();
    for (let t = 0; t < model.ntendon; t++) {
      let startW = data.ten_wrapadr[t];
      let r = model.tendon_width[t];
      for (let w = startW; w < startW + data.ten_wrapnum[t] -1 ; w++) {
        let tendonStart = getPosition(data.wrap_xpos, w    , new THREE.Vector3());
        let tendonEnd   = getPosition(data.wrap_xpos, w + 1, new THREE.Vector3());
        let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

        let validStart = tendonStart.length() > 0.01;
        let validEnd   = tendonEnd  .length() > 0.01;

        if (validStart) { mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, identityQuat, new THREE.Vector3(r, r, r))); }
        if (validEnd  ) { mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , identityQuat, new THREE.Vector3(r, r, r))); }
        if (validStart && validEnd) {
          mat.compose(tendonAvg, identityQuat.setFromUnitVectors(
            new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
            new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
          mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
          numWraps++;
        }
      }
    }

    let curFlexSphereInd = numWraps;
    let tempvertPos = new THREE.Vector3();
    let tempvertRad = new THREE.Vector3();
    for (let i = 0; i < model.nflex; i++) {
      for(let j = 0; j < model.flex_vertnum[i]; j++) {
        let vertIndex = model.flex_vertadr[i] + j;
        getPosition(data.flexvert_xpos, vertIndex, tempvertPos);
        let r   = 0.01;
        mat.compose(tempvertPos, identityQuat, tempvertRad.set(r, r, r));

        mujocoRoot.spheres.setMatrixAt(curFlexSphereInd, mat);
        curFlexSphereInd++;
      }
    }
    mujocoRoot.cylinders.count = numWraps;
    mujocoRoot.spheres  .count = curFlexSphereInd;
    mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
    mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
  }
}

/** Downloads the scenes/assets folder to MuJoCo's virtual filesystem
 * @param {mujoco} mujoco */

export function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]);
  } else {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 1],
       buffer[(index * 3) + 2]);
  }
}

/** Access the quaternion at index, swizzle for three.js, and apply to the target THREE.Quaternion
 * @param {Float32Array|Float64Array} buffer
 * @param {number} index
 * @param {THREE.Quaternion} target */
export function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
       buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]);
  } else {
    return target.set(
       buffer[(index * 4) + 0],
       buffer[(index * 4) + 1],
       buffer[(index * 4) + 2],
       buffer[(index * 4) + 3]);
  }
}

/** Converts this Vector3's Handedness to MuJoCo's Coordinate Handedness
 * @param {THREE.Vector3} target */
export function toMujocoPos(target) { return target.set(target.x, -target.z, target.y); }

