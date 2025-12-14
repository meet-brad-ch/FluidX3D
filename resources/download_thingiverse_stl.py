#!/usr/bin/env python3
"""
Download STL files from Thingiverse for FluidX3D examples.

Thingiverse provides zip archives, not direct STL downloads.
Thingiverse uses JavaScript countdown timers before downloads, requiring browser automation.
This script uses Playwright to handle the download process.

Requirements:
    pip install playwright
    playwright install chromium
"""

import argparse
import os
import sys
import tempfile
import zipfile
from pathlib import Path


def download_thingiverse_zip_playwright(thing_url, download_dir, timeout=120000):
    """
    Download zip from Thingiverse using Playwright (handles JavaScript countdown).

    Args:
        thing_url: Thingiverse thing files URL (e.g., https://www.thingiverse.com/thing:182114/files)
        download_dir: Directory where browser will download the zip
        timeout: Maximum milliseconds to wait for download (default 60000 = 60s)

    Returns:
        Path to downloaded zip file, or None if failed
    """
    import time
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("✗ Playwright not installed.", file=sys.stderr)
        print("  Install with:", file=sys.stderr)
        print("    pip install playwright", file=sys.stderr)
        print("    playwright install chromium", file=sys.stderr)
        return None

    print(f"Using Playwright to download from: {thing_url}")
    print(f"Download directory: {download_dir}")

    with sync_playwright() as p:
        try:
            # Launch browser with stealth settings to bypass Cloudflare
            browser = p.chromium.launch(
                headless=True,
                args=[
                    '--disable-blink-features=AutomationControlled',
                    '--no-sandbox',
                    '--disable-dev-shm-usage',
                ]
            )
            context = browser.new_context(
                accept_downloads=True,
                user_agent='Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36',
                viewport={'width': 1920, 'height': 1080},
                locale='en-US',
            )

            # Add stealth script to hide automation
            context.add_init_script("""
                Object.defineProperty(navigator, 'webdriver', {get: () => undefined});
            """)

            page = context.new_page()

            print("✓ Browser launched")

            # Navigate to page
            print("Loading Thingiverse page...")
            page.goto(thing_url, timeout=timeout, wait_until="domcontentloaded")
            time.sleep(2)  # Give page time to render

            print("✓ Page loaded")

            # Find download button - try multiple selectors
            print("Looking for download button...")
            download_button = None

            # Try different selectors
            selectors = [
                "text='Download All Files'",
                "button:has-text('Download')",
                "a:has-text('Download All')",
                "[class*='download']",
            ]

            for selector in selectors:
                try:
                    download_button = page.locator(selector).first
                    if download_button.is_visible(timeout=2000):
                        print(f"✓ Found download button with selector: {selector}")
                        break
                except:
                    continue

            if not download_button or not download_button.is_visible():
                print("✗ Could not find download button", file=sys.stderr)
                print("  Page title:", page.title(), file=sys.stderr)
                browser.close()
                return None

            # Start waiting for download before clicking
            print("Clicking download button...")
            with page.expect_download(timeout=timeout) as download_info:
                download_button.click()
                print("Waiting for countdown timer and download...")

            download = download_info.value
            print(f"✓ Download started: {download.suggested_filename}")

            # Save to download directory
            download_path = os.path.join(download_dir, download.suggested_filename)
            download.save_as(download_path)

            print(f"✓ Download complete: {download_path}")
            file_size = os.path.getsize(download_path)
            print(f"  Size: {file_size / 1024 / 1024:.2f} MB")

            browser.close()
            return download_path

        except Exception as e:
            print(f"✗ Playwright error: {e}", file=sys.stderr)
            try:
                browser.close()
            except:
                pass
            return None


def extract_stl_from_zip(zip_path, stl_filename, output_dir):
    """Extract a specific STL file from a zip archive."""
    print(f"\nExtracting '{stl_filename}' from zip archive...")

    try:
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            # List all files in the zip
            all_files = zip_ref.namelist()
            print(f"Zip contains {len(all_files)} files")

            # Find the STL file (may be in subdirectory)
            stl_file = None
            for file in all_files:
                if file.endswith(stl_filename):
                    stl_file = file
                    break

            if not stl_file:
                # Try case-insensitive search
                stl_lower = stl_filename.lower()
                for file in all_files:
                    if file.lower().endswith(stl_lower):
                        stl_file = file
                        break

            if not stl_file:
                print(f"✗ STL file '{stl_filename}' not found in archive", file=sys.stderr)
                print(f"Available STL files:", file=sys.stderr)
                for f in all_files:
                    if f.endswith('.stl'):
                        print(f"  - {f}", file=sys.stderr)
                return False

            print(f"Found: {stl_file}")

            # Extract to output directory
            output_path = os.path.join(output_dir, stl_filename)

            # Read from zip and write to output
            with zip_ref.open(stl_file) as source:
                with open(output_path, 'wb') as target:
                    target.write(source.read())

            file_size = os.path.getsize(output_path)
            print(f"✓ Extracted: {file_size / 1024 / 1024:.2f} MB")
            print(f"✓ Saved to: {output_path}")
            return True

    except zipfile.BadZipFile:
        print(f"✗ Invalid zip file: {zip_path}", file=sys.stderr)
        return False
    except Exception as e:
        print(f"✗ Extraction error: {e}", file=sys.stderr)
        return False


def download_thingiverse_stl(thing_id, stl_filename, output_dir):
    """
    Download an STL file from Thingiverse.

    Args:
        thing_id: Thingiverse thing ID (e.g., "182114")
        stl_filename: Name of the STL file to extract (e.g., "Cow_t.stl")
        output_dir: Directory to save the STL file
    """
    print(f"=== Downloading Thingiverse Thing {thing_id} ===")
    print(f"Target STL: {stl_filename}")
    print()

    # Create output directory if needed
    os.makedirs(output_dir, exist_ok=True)

    # Check if STL already exists and is valid
    output_stl_path = os.path.join(output_dir, stl_filename)
    if os.path.exists(output_stl_path):
        file_size = os.path.getsize(output_stl_path)
        if file_size > 1024:  # More than 1KB
            print(f"✓ STL file already exists: {output_stl_path}")
            print(f"  Size: {file_size / 1024 / 1024:.2f} MB")
            print("  Use --force to re-download")
            return True

    # Thingiverse files page URL
    thing_url = f"https://www.thingiverse.com/thing:{thing_id}/files"

    # Create temporary download directory
    with tempfile.TemporaryDirectory() as tmp_download_dir:
        # Download the zip using Playwright
        zip_path = download_thingiverse_zip_playwright(thing_url, tmp_download_dir)
        if not zip_path:
            print("\n✗ Failed to download zip", file=sys.stderr)
            print("  Try manual download from:", file=sys.stderr)
            print(f"  {thing_url}", file=sys.stderr)
            return False

        # Extract the STL
        if not extract_stl_from_zip(zip_path, stl_filename, output_dir):
            return False

    print("\n✓ Success!")
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Download STL files from Thingiverse for FluidX3D examples',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --thing 182114 --stl Cow_t.stl --output ./stl
  %(prog)s -t 2772812 -s techtris_airplane.stl -o ./stl
        """
    )

    parser.add_argument('-t', '--thing', required=True,
                        help='Thingiverse thing ID (e.g., 182114)')
    parser.add_argument('-s', '--stl', required=True,
                        help='STL filename to extract (e.g., Cow_t.stl)')
    parser.add_argument('-o', '--output', required=True,
                        help='Output directory for STL file')
    parser.add_argument('-f', '--force', action='store_true',
                        help='Force re-download even if file exists')

    args = parser.parse_args()

    # Remove existing file if --force
    if args.force:
        output_path = os.path.join(args.output, args.stl)
        if os.path.exists(output_path):
            print(f"Removing existing file: {output_path}")
            os.remove(output_path)

    success = download_thingiverse_stl(args.thing, args.stl, args.output)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
