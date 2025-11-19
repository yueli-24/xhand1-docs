import { describe, it, expect, beforeAll } from 'vitest';
import { drizzle } from 'drizzle-orm/mysql2';
import { admins, docContents } from '../drizzle/schema';
import { eq } from 'drizzle-orm';
import * as crypto from 'crypto';

// Simple password hashing
function hashPassword(password: string): string {
  return crypto.createHash("sha256").update(password).digest("hex");
}

describe('Admin System', () => {
  let db: ReturnType<typeof drizzle>;

  beforeAll(async () => {
    // Initialize database connection
    if (!process.env.DATABASE_URL) {
      throw new Error('DATABASE_URL not set');
    }
    db = drizzle(process.env.DATABASE_URL);
  });

  it('should have admin user created', async () => {
    const admin = await db.select().from(admins).where(eq(admins.username, 'tsuki')).limit(1);
    
    expect(admin.length).toBe(1);
    expect(admin[0].username).toBe('tsuki');
  });

  it('should verify admin password', async () => {
    const admin = await db.select().from(admins).where(eq(admins.username, 'tsuki')).limit(1);
    
    const passwordHash = hashPassword('1Q2w3E4r');
    expect(admin[0].passwordHash).toBe(passwordHash);
  });

  it('should be able to create and retrieve document content', async () => {
    const testDocId = 'test-doc-' + Date.now();
    const testContent = '# Test Document\n\nThis is a test document.';
    
    // Insert test document
    await db.insert(docContents).values({
      docId: testDocId,
      content: testContent,
      updatedBy: 1,
    });
    
    // Retrieve document
    const doc = await db.select().from(docContents).where(eq(docContents.docId, testDocId)).limit(1);
    
    expect(doc.length).toBe(1);
    expect(doc[0].docId).toBe(testDocId);
    expect(doc[0].content).toBe(testContent);
    
    // Clean up
    await db.delete(docContents).where(eq(docContents.docId, testDocId));
  });

  it('should be able to update document content', async () => {
    const testDocId = 'test-doc-update-' + Date.now();
    const initialContent = '# Initial Content';
    const updatedContent = '# Updated Content\n\nThis content has been updated.';
    
    // Insert test document
    await db.insert(docContents).values({
      docId: testDocId,
      content: initialContent,
      updatedBy: 1,
    });
    
    // Update document
    await db.update(docContents)
      .set({ content: updatedContent })
      .where(eq(docContents.docId, testDocId));
    
    // Retrieve updated document
    const doc = await db.select().from(docContents).where(eq(docContents.docId, testDocId)).limit(1);
    
    expect(doc[0].content).toBe(updatedContent);
    
    // Clean up
    await db.delete(docContents).where(eq(docContents.docId, testDocId));
  });
});
