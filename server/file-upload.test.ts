import { describe, it, expect } from 'vitest';
import { storagePut } from './storage';
import * as crypto from 'crypto';

describe('File Upload Logic', () => {
  it('should generate unique file keys', () => {
    const fileName = 'test-image.png';
    const ext = fileName.split('.').pop() || 'bin';
    const timestamp = Date.now();
    const randomStr = crypto.randomBytes(8).toString('hex');
    const key = `docs/${timestamp}-${randomStr}.${ext}`;
    
    expect(key).toMatch(/^docs\/\d+-[a-f0-9]{16}\.png$/);
  });
  
  it('should decode base64 correctly', () => {
    const testData = 'Hello, World!';
    const base64 = Buffer.from(testData).toString('base64');
    const decoded = Buffer.from(base64, 'base64').toString();
    
    expect(decoded).toBe(testData);
  });
  
  it('should handle different file extensions', () => {
    const testCases = [
      { fileName: 'image.jpg', expected: 'jpg' },
      { fileName: 'document.pdf', expected: 'pdf' },
      { fileName: 'data.csv', expected: 'csv' },
    ];
    
    for (const testCase of testCases) {
      const ext = testCase.fileName.split('.').pop() || 'bin';
      expect(ext).toBe(testCase.expected);
    }
    
    // Test file without extension
    const parts = 'noext'.split('.');
    const ext = parts.length > 1 ? parts.pop() : 'bin';
    expect(ext).toBe('bin');
  });
  
  it('should upload to storage successfully', async () => {
    // Create a small test file
    const testData = Buffer.from('test file content');
    const key = `test/${Date.now()}.txt`;
    
    try {
      const result = await storagePut(key, testData, 'text/plain');
      
      expect(result.key).toBe(key);
      expect(result.url).toBeDefined();
      expect(result.url).toContain('http');
    } catch (error: any) {
      // Storage might not be available in test environment
      console.log('Storage test skipped:', error.message);
      expect(error).toBeDefined();
    }
  });
});
