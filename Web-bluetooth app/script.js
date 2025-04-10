let x, y, z, xg, yg, zg;
let ledState = false;

let xCharacteristic, yCharacteristic, zCharacteristic, ledCharacteristic, xgCharacteristic, ygCharacteristic, zgCharacteristic;

const serviceUUID = "19b10000-e8f2-537e-4f6c-d104768a1214";

const xCharacteristicUUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
const yCharacteristicUUID = "19b10002-e8f2-537e-4f6c-d104768a1214";
const zCharacteristicUUID = "19b10003-e8f2-537e-4f6c-d104768a1214";

const ledCharacteristicUUID = "19b10004-e8f2-537e-4f6c-d104768a1214";

const xgCharacteristicUUID = "19b10005-e8f2-537e-4f6c-d104768a1214";
const ygCharacteristicUUID = "19b10006-e8f2-537e-4f6c-d104768a1214";
const zgCharacteristicUUID = "19b10007-e8f2-537e-4f6c-d104768a1214";

async function connect(){

    const device = await navigator.bluetooth.requestDevice({filters: [{services: [serviceUUID]}]});
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(serviceUUID);

    xCharacteristic = await service.getCharacteristic(xCharacteristicUUID);
    yCharacteristic = await service.getCharacteristic(yCharacteristicUUID);
    zCharacteristic = await service.getCharacteristic(zCharacteristicUUID);
    
    ledCharacteristic = await service.getCharacteristic(ledCharacteristicUUID);
    
    xgCharacteristic = await service.getCharacteristic(xgCharacteristicUUID);
    ygCharacteristic = await service.getCharacteristic(ygCharacteristicUUID);
    zgCharacteristic = await service.getCharacteristic(zgCharacteristicUUID);

    await xCharacteristic.startNotifications();
    await yCharacteristic.startNotifications();
    await zCharacteristic.startNotifications();
    
    await xgCharacteristic.startNotifications();
    await ygCharacteristic.startNotifications();
    await zgCharacteristic.startNotifications();

    xCharacteristic.addEventListener('characteristicvaluechanged', readX);
    yCharacteristic.addEventListener('characteristicvaluechanged', readY);
    zCharacteristic.addEventListener('characteristicvaluechanged', readZ);
    
    xgCharacteristic.addEventListener('characteristicvaluechanged', readXG);
    ygCharacteristic.addEventListener('characteristicvaluechanged', readYG);
    zgCharacteristic.addEventListener('characteristicvaluechanged', readZG);

}

function readX(event) {
    x = event.target.value.getFloat32(0, true);
    x = parseFloat(x.toFixed(2));
    document.getElementById('x').textContent = x;
}

function readY(event) {
    y = event.target.value.getFloat32(0, true);
    y = parseFloat(y.toFixed(2));
    document.getElementById('y').textContent = y;
}

function readZ(event) {
    z = event.target.value.getFloat32(0, true);
    z = parseFloat(z.toFixed(2));
    document.getElementById('z').textContent = z;
} 

function readXG(event) {
    xg = event.target.value.getFloat32(0, true);
    xg = parseFloat(xg.toFixed(2));
    document.getElementById('xg').textContent = xg;
}

function readYG(event) {
    yg = event.target.value.getFloat32(0, true);
    yg = parseFloat(yg.toFixed(2));
    document.getElementById('yg').textContent = yg;
}

function readZG(event) {
    zg = event.target.value.getFloat32(0, true);
    zg = parseFloat(zg.toFixed(2));
    document.getElementById('zg').textContent = zg;
} 

async function toggleLED(){
    ledState = !ledState;

    let buffer = new ArrayBuffer(1);
    let view = new Uint8Array(buffer);
    view[0] = ledState;

    await ledCharacteristic.writeValue(view);
}

document.getElementById('connect').addEventListener("click", connect);
document.getElementById('led').addEventListener("click", toggleLED);