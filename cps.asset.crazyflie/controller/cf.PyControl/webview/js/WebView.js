class WebView {
    constructor() {
        this.picsUrls = new Array();
        this.uavNames = new Array();
        this.divArray = new Array();
        this.canvasArray = new Array();
        this.removedPics = new Array();
        this.currendPics = new Array();
        this.checkComplet = false;
        this.updateComplet = false;
        this.findNewPicture = false;
        this.checkCount = 100;
        this.onUdate = false;
        this.canvasHeight = 710;
        this.canvasWidth = 1900;
        this.divWidth = 122;
        this.colorSaved = false;
        this.backgroundColorWhite = null;
        this.borderColorBlack = null;
        this.backgroundColorGrey = "#B0B0B0";
        this.borderColorRed = "red";
        
        // 从URL获取无人机ID
        this.droneId = this.getDroneIdFromUrl();
        this.createStartStopCallback();
        
        // 添加更新间隔时间（毫秒）
        this.updateInterval = 1000;
        this.updateTimer = null;
    }

    getDroneIdFromUrl() {
        const explicitDrone = new URLSearchParams(window.location.search).get('drone');
        if (explicitDrone && /^cf\d+$/.test(explicitDrone)) return explicitDrone;
        // 从URL中提取端口号
        const port = window.location.port;
        const numericPort = Number(port);
        const mapping = window.CF_RUNTIME;
        const offset = mapping ? numericPort - mapping.web_port : -1;
        if (mapping && offset >= 0 && offset < mapping.drone_ids.length) return `cf${mapping.drone_ids[offset]}`;
        if (numericPort >= 8080 && numericPort < 8092) return `cf${231 + numericPort - 8080}`;
        // 根据端口号映射到无人机ID
        const portToDroneId = {
            '8080': 'cf231',
            '8081': 'cf232',
            '8082': 'cf233',
            '8083': 'cf234',
            '8084': 'cf235',
            '8085': 'cf236',
            '8086': 'cf237',
            '8087': 'cf238'
            // 可以在这里添加更多的端口和无人机ID映射
        };
        return portToDroneId[port] || 'cf231'; // 默认返回cf231
    }

    createStartStopCallback() {
        const button = document.getElementById("button_start_stop");
        const constThis = this;
        button.onclick = async function() {
            if (constThis.onUdate) {
                clearInterval(constThis.updateTimer);
                constThis.onUdate = false;
                button.innerHTML = "&nbsp;Start Update&nbsp;";
            } else {
                constThis.onUdate = true;
                button.innerHTML = "&nbsp;Stop Update&nbsp;";
                // 立即执行一次更新
                await constThis.updateData();
                // 设置定时器持续更新
                constThis.updateTimer = setInterval(async () => {
                    if (constThis.onUdate) {
                        await constThis.updateData();
                    }
                }, constThis.updateInterval);
            }
        };
    }

    clearData(alsoUrls) {
        if(alsoUrls) {
            var element;
            for (let i = 0; i < this.divArray.length; i++) {
                element = document.getElementById(this.divArray.at(i));
                if(element != null) {
                    console.log("ok div", this.divArray.at(i));
                    element.remove();
                }
            }
            for (let i = 0; i < this.canvasArray.length; i++) {
                element = document.getElementById(this.canvasArray.at(i));
                if(element != null) {
                    console.log("ok canvas", this.canvasArray.at(i));
                    element.remove();
                }
            }
            for (let i = 0; i < this.uavNames.length; i++) {
                element = document.getElementById(this.uavNames.at(i));
                if(element != null) {
                    console.log("ok pic", this.uavNames.at(i));
                    element.remove();
                }
            }
            this.divArray = null;
            this.divArray = new Array();
            this.canvasArray = null;
            this.canvasArray = new Array();
            this.uavNames = null;
            this.uavNames = new Array();
            this.picsUrls = null;
            this.picsUrls = new Array();
            this.removedPics = null;
            this.removedPics = new Array();    
            return;
        }

        var findRemovedDiv = false;
        var findRemovedCanvas = false;
        var findRemovedPic = false;
        var countRemoved = 0;
        for (let i = 0; i < this.divArray.length; i++) {
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findRemovedPic = true;
                }
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i) + "_div") == 0) {
                    findRemovedDiv = true;
                }
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i) + "_canvas") == 0) {
                    findRemovedCanvas = true;
                }
            }
            if(findRemovedDiv) {
                findRemovedDiv = false;
                countRemoved++;
            } else {
                document.getElementById(this.divArray.at(i)).remove();
            }
            if(findRemovedCanvas) {
                findRemovedCanvas = false;
                countRemoved++;
            } else {
                document.getElementById(this.canvasArray.at(i)).remove();
            }
            if(findRemovedPic) {
                findRemovedPic = false;
                countRemoved++;
            } else {
                document.getElementById(this.uavNames.at(i)).remove();
            }
        }
        var findRemoved;
        for(let i = 0; i < this.divArray.length; i++) {
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.divArray.at(i).localeCompare(this.removedPics.at(i2) + "_div") == 0) {
                    findRemoved = true;
                }
            } 
            if(!findRemoved) {
                this.divArray.splice(i, 1);
            }
        }
        for(let i = 0; i < this.canvasArray.length; i++) {
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.canvasArray.at(i).localeCompare(this.removedPics.at(i2) + "_canvas") == 0) {
                    findRemoved = true;
                }
            } 
            if(!findRemoved) {
                this.canvasArray.splice(i, 1);
            }
        }
    }

    async updateData() {
        this.getPicsUrlsAndNames();
        while (!this.checkComplet) {
            await new Promise(resolve => setTimeout(resolve, 20));
        }
        this.checkComplet = false;
        if(this.findNewPicture) {
            this.outputTextInfo((this.picsUrls.length + " SM's finding: "), true);
            this.clearData(false);
            for (let i = 0; i < this.picsUrls.length; i++) {
                this.outputTextInfo(this.picsUrls.at(i), false);
                this.loadSMPictuere(this.picsUrls.at(i), this.uavNames.at(i), (80 + (i * this.canvasHeight) + (i * 5)));
            }
            this.findNewPicture = false;
        } else {
            this.updateSMPictuere();
        }
        await new Promise(resolve => setTimeout(resolve, 300));

        this.updateComplet = true;
    }

    updateSMPictuere() {
        var imgs = new Array(this.uavNames.length);
        var div = new Array(this.uavNames.length);
        var canvas = new Array(this.uavNames.length);
        var findRemoved;
        for(let i = 0; i < this.uavNames.length; i++) {
            imgs[i] = document.getElementById(this.uavNames.at(i));
            imgs[i].crossOrigin = "Anonymous";  // 添加跨域支持
            imgs[i].src = this.picsUrls.at(i) + "?" + Date.now().toString() + "&r=" + Math.random();
            
            console.log("Updating image: " + this.picsUrls.at(i));
            
            // 使用箭头函数保持this上下文
            imgs[i].onload = () => {
                console.log("Image updated successfully: " + this.picsUrls.at(i));
                try {
                    var canvas = document.getElementById(this.uavNames.at(i) + "_canvas");
                    var ctx = canvas.getContext("2d");
                    
                    // 设置所需的宽度或高度以进行缩放
                    var desiredWidth = 1900;
                    var desiredHeight;
                    // 计算宽高比
                    var aspectRatio = imgs[i].width / imgs[i].height;
                    
                    // 计算新高度以保持宽高比
                    desiredHeight = desiredWidth / aspectRatio;
                    
                    // 在绘制前清除画布
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                    
                    // 用新尺寸绘制图像
                    ctx.drawImage(imgs[i], 0, 0, desiredWidth, desiredHeight);
                } catch (error) {
                    console.error("Error updating image:", error);
                }
            };
            
            imgs[i].onerror = () => {
                console.error("Failed to update image:", this.picsUrls.at(i));
            };

            div[i] = document.getElementById(this.uavNames.at(i) + "_div");
            canvas[i] = document.getElementById(this.uavNames.at(i) + "_canvas");
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findRemoved = true;
                }
            }
            if(findRemoved) {
                div[i].style.backgroundColor = this.backgroundColorGrey;
                div[i].style.borderColor = this.borderColorRed;
                canvas[i].style.borderColor = this.borderColorRed;
            } else {
                div[i].style.backgroundColor = this.backgroundColorWhite;
                div[i].style.borderColor = this.borderColorBlack;
                canvas[i].style.borderColor = this.borderColorBlack;
            }
        }
    }

    loadSMPictuere(imagePath, uavName, topPose) {
        console.log(`Loading image: ${imagePath}`);
        
        var div = document.createElement('div');
        div.id = uavName + "_div";
        div.width = this.divWidth;
        div.height = this.canvasHeight;
        div.style.position = "absolute";
        div.style.top = topPose.toString() + "px";
        div.style.left = "10px"
        div.style.border = "1px solid";
        div.style.padding = "25px";
        div.style.fontSize = "25px";
        div.innerHTML = uavName;
        this.divArray.push(div.id);

        var canvas = document.createElement('canvas');
        canvas.id = uavName + "_canvas";
        canvas.width = this.canvasWidth;
        canvas.height = this.canvasHeight;
        canvas.style.position = "absolute";
        canvas.style.top = topPose.toString() + "px";
        canvas.style.left = (this.divWidth + 10).toString() + "px";
        canvas.style.border = "1px solid";
        this.canvasArray.push(canvas.id);
        
        var body = document.getElementsByTagName("body")[0];
        body.appendChild(canvas);
        body.appendChild(div);

        var ctx = canvas.getContext("2d");
        var img = new Image();
        var timestamp = Date.now().toString();
        img.crossOrigin = "Anonymous";  // 添加跨域支持
        
        // 添加随机参数防止缓存
        const fullUrl = imagePath + "?t=" + timestamp + "&r=" + Math.random();
        console.log(`Loading image with URL: ${fullUrl}`);
        img.src = fullUrl;
        
        const constThis = this;
        img.onload = function () {
            try {
                console.log(`Image loaded successfully: ${imagePath} (size: ${img.width}x${img.height})`);
                
                // Set the desired width or height for scaling
                var desiredWidth = 1900;
                var desiredHeight;
                // Calculate the aspect ratio
                var aspectRatio = img.width / img.height;

                // Calculate the new height to maintain the aspect ratio
                desiredHeight = desiredWidth / aspectRatio;

                // Clear the canvas before drawing
                ctx.clearRect(0, 0, canvas.width, canvas.height);

                // Draw the image with the new dimensions
                ctx.drawImage(img, 0, 0, desiredWidth, desiredHeight);
                
                // 在canvas上显示调试信息
                ctx.fillStyle = "red";
                ctx.font = "14px Arial";
                ctx.fillText(`Image loaded: ${imagePath} at ${timestamp}`, 10, 20);
            } catch (error) {
                console.error("Error in drawing image: ", error);
                ctx.fillStyle = "red";
                ctx.font = "20px Arial";
                ctx.fillText("Error loading image: " + error.message, 10, 40);
            }
        };
        
        img.onerror = function(e) {
            console.error(`Failed to load image: ${imagePath}`, e);
            ctx.fillStyle = "red";
            ctx.font = "20px Arial";
            ctx.fillText(`Error loading image: ${imagePath}`, 10, 40);
            ctx.fillText("Please check if the image exists and has proper permissions", 10, 70);
        };

        img.id = uavName;
        img.style.display = "none";
        body.appendChild(img);

        if(!this.colorSaved) {
            this.backgroundColorWhite = div.style.backgroundColor;
            this.borderColorBlack = canvas.style.borderColor;
            this.colorSaved = true;
        } else {
            for(let i = 0; i < this.removedPics.length; i++) {
                if(this.removedPics.at(i).localeCompare(uavName) == 0) {
                    div.style.backgroundColor = this.backgroundColorGrey;
                    div.style.borderColor = this.borderColorRed;
                    canvas.style.borderColor = this.borderColorRed;
                    return;
                }
                div.style.backgroundColor = this.backgroundColorWhite;
                div.style.borderColor = this.borderColorBlack;
                canvas.style.borderColor = this.borderColorBlack;
            }
        }
    }

    async getPicsUrlsAndNames() {
        // 只获取当前无人机ID对应的图片
        const imgDir = `img/${this.droneId}`;
        try {
            // 获取最新的图片编号
            const response = await fetch(`${imgDir}/latest.txt`);
            const latestNumber = await response.text();
            const imageNumber = parseInt(latestNumber);
            
            console.log(`Latest image number from file: ${imageNumber}`);
            
            if (!isNaN(imageNumber)) {
                // 尝试加载最新的图片
                let foundImage = false;
                for (let i = imageNumber; i >= 1; i--) {
                    const imagePath = `${imgDir}/uav${i}.png`;
                    console.log(`Trying to load image: ${imagePath}`);
                    
                    // 检查图片是否存在
                    const imgResponse = await fetch(imagePath);
                    if (imgResponse.ok) {
                        console.log(`Image found: ${imagePath}`);
                        this.picsUrls = [imagePath];
                        this.uavNames = [this.droneId];
                        this.findNewPicture = true;
                        foundImage = true;
                        break;
                    }
                }
                
                if (!foundImage) {
                    console.error('No valid images found in the directory');
                }
            } else {
                console.error('Invalid image number in latest.txt');
            }
        } catch (error) {
            console.error('Error fetching images:', error);
        }
        
        this.checkComplet = true;
    }

    updateRemovedPics() {
        var findMatch;
        var findRemoved;
        for(let i = 0; i < this.uavNames.length; i++) {
            findMatch = false;
            for(let i2 = 0; i2 < this.currendPics.length; i2++) {
                if(this.currendPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findMatch = true;
                }
            }
            if(!findMatch) {
                findRemoved = false;
                for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                    if(this.uavNames.at(i).localeCompare(this.removedPics.at(i2)) == 0) {
                        findRemoved = true;
                    }
                }
                if(!findRemoved) {
                    this.removedPics.push(this.uavNames.at(i));
                }
            }
        }
        for(let i = 0; i < this.removedPics.length; i++) {
            for(let i2 = i+1; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i).localeCompare(this.removedPics.at(i2)) == 0) {
                    this.removedPics.splice(i2, 1);
                }
            }
            for(let i2 = 0; i2 < this.currendPics.length; i2++) {
                if(this.currendPics.at(i2).localeCompare(this.removedPics.at(i)) == 0) {
                    this.removedPics.splice(i, 1);
                }
            }
        }
        //console.log(this.removedPics);
    }

    async getPicUrlOrNull(url) {
        return new Promise((resolve, reject) => {
            const img = new Image();
            img.crossOrigin = "Anonymous";  // 添加跨域支持
            
            // 添加随机参数防止缓存
            const fullUrl = url + "?t=" + Date.now() + "&r=" + Math.random();
            console.log("Checking image URL: " + fullUrl);
            
            img.onload = () => {
                console.log("Image check successful: " + url + " (size: " + img.width + "x" + img.height + ")");
                resolve(url);
            };
            
            img.onerror = (error) => {
                console.error("Image check failed for url: " + url, error);
                reject(`Image not found for url ${url}`);
            };
            
            // 设置超时
            const timeout = setTimeout(() => {
                console.warn("Image load timeout for: " + url);
                reject(`Image load timeout for ${url}`);
            }, 5000);
            
            // 清除超时
            img.onload = () => {
                clearTimeout(timeout);
                console.log("Image check successful: " + url);
                resolve(url);
            };
            
            img.src = fullUrl;
        }).catch((error) => {
            console.error("Error loading image:", error);
            return null;
        });
    }

    async outputTextInfo(infoText, clear) {
        var tArea = document.getElementById('text_infos_output');
        if (clear) {
            tArea.value = infoText;
        } else {
            var oldTxt = tArea.value;
            tArea.value = oldTxt + "\r\n" + infoText;
        }
        tArea.scrollTop = tArea.scrollHeight;
    }
}
