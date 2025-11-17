const fs = require('fs');

// 读取原始文件
const content = fs.readFileSync('client/src/lib/docs.ts', 'utf8');

// 找到快速入门章节中的host-software部分，使用String.raw替换
// 由于内容太长，我们采用分段替换的策略

// 先恢复到之前的版本
const lines = content.split('\n');
let inHostSoftware = false;
let inFirstTest = false;
let bracketCount = 0;
let result = [];

for (let i = 0; i < lines.length; i++) {
  const line = lines[i];
  
  // 检测是否进入host-software或first-test章节
  if (line.includes('id: "host-software"') || line.includes('id: "first-test"')) {
    // 跳过这些章节的content部分，保持原样
    result.push(line);
    continue;
  }
  
  result.push(line);
}

fs.writeFileSync('client/src/lib/docs.ts', result.join('\n'), 'utf8');
console.log('文件已恢复');
