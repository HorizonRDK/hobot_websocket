// 'use strict';
let AwesomeMessage = null;

function protobufInit() {
    protobuf.load('../../x3/protos/x3.proto', function (err, root) {
        if (err) throw err;
        AwesomeMessage = root.lookupType('x3.FrameMessage');
    });
}
protobufInit();
/**
 * 数据转换
 * @param {*} buffer
 */
function transformData(buffer) {
    let startTime = new Date().valueOf();
    let unit8Array = new Uint8Array(buffer);
    let message = AwesomeMessage.decode(unit8Array);
    let object = AwesomeMessage.toObject(message);
    let image = object['img_']; // 图片
    let smartMsg = object['smartMsg_']; // 智能数据
    let statisticsMsg = object['StatisticsMsg_']; // 性能数据
    // console.log('smartMsg---', smartMsg)
    let imageBlob = undefined;
    let bodyAction = undefined;
    let bodyPoints = [];

    // 分类转换智能数据：box 为框、points 为骨骼线、info 为局部遮挡、attributes 为属性展示
    if (typeof smartMsg !== 'undefined') {
        let targets = smartMsg['targets_'];
        if (typeof targets !== 'undefined' && targets.length > 0) {
            let onetarget = targets[0]
            let kpsPoints = onetarget['points_']
            let actions = onetarget['attributes_']
            if (kpsPoints !== 'undefined' && kpsPoints.length > 0) {
                kpsPoints = kpsPoints.map(function (item) {
                    //关节点数据
                    let originalPoints = item['points_'];
                    let skeletonPoints = {}
                    originalPoints.forEach(function (item, index) {
                        let key = Config.skeletonKey[index];
                        skeletonPoints[key] = {
                            x: item['x_'],
                            y: item['y_'],
                            score: item['score_']
                        };
                    });
                    bodyPoints.push(skeletonPoints);
                })
            }
            if (actions !== 'undefined' && actions.length > 0) {
                bodyAction = actions[0]['value_'];
                console.log("main: action = ", bodyAction);
            }
        }
    }
    return {
        bodyAction,
        bodyPoints
    }
}


function getURLParameter() {
    let socketIP = getUrlQueryParameter('ws');
    let port = getUrlQueryParameter('port');
    let netId = getUrlQueryParameter('netid');
    let cameraId = getUrlQueryParameter('camera');
    let id = getUrlQueryParameter('id');

    let doc_socketIP = getUrlQueryParameter('doc_ws');
    let doc_port = getUrlQueryParameter('doc_port');
    let cam_id = getUrlQueryParameter('cam_id');

    return {
        socketIP,
        port,
        netId,
        cameraId,
        id,
        doc_socketIP,
        doc_port,
        cam_id
    }
}