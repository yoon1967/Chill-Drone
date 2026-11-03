import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('recent cards open completed depth maps; hover trash deletes only the selected card; folder chooser opens directly',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      const jobs = ['one', 'two'].map((name, i) => ({id: `import_${String(i + 1).repeat(16)}`, name,
        status: 'complete', options: {mode: 'depth_rgbd'}, summary: {kind: 'result', name, points: 3, keyframes: 2}}));
      let items = [...jobs]; const deleted = [];
      await page.route('**/api/import/library', route => route.fulfill({json: {items}}));
      await page.route('**/api/import/delete', route => {
        const id = route.request().postDataJSON().local_id;
        deleted.push(id); items = items.filter(job => job.id !== id);
        return route.fulfill({json: {status: 'deleted'}});
      });
      await page.route('**/api/import/local', route => route.fulfill({json: jobs.find(job => job.id === route.request().postDataJSON().local_id)}));
      await page.goto(url + '/#saved');
      await page.locator('.saved-record-card').first().waitFor();
      const trash = page.locator('.saved-record-delete').first();
      assert.equal(await trash.evaluate(node => getComputedStyle(node).opacity), '0');
      await page.locator('.saved-record-card').first().hover();
      assert.equal(await trash.evaluate(node => getComputedStyle(node).opacity), '1');
      assert.ok((await trash.boundingBox()).width >= 44, 'trash has a comfortable click target');
      assert.ok((await trash.locator('svg').boundingBox()).width >= 24, 'trash icon is readable');
      await trash.click();
      await page.waitForFunction(() => document.querySelectorAll('.saved-record-card').length === 1);
      assert.deepEqual(deleted, [jobs[0].id]);
      assert.ok(!page.url().includes('asset='), 'trash must not open the card');
      const chooser = page.waitForEvent('filechooser');
      await page.locator('#saved-existing').click();
      assert.equal((await chooser).isMultiple(), true);
      assert.equal(await page.locator('#saved-existing-tools').isVisible(), false);
      await page.locator('.saved-record').click();
      await page.waitForURL(`**/?asset=${jobs[1].id}#saved`);
    } finally { await browser.close(); }
  });
