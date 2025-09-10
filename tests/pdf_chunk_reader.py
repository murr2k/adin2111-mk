#!/usr/bin/env python3
"""
PDF Chunk Reader - Process large PDF files in manageable chunks
Enables Claude Code to analyze large PDFs by reading them in smaller sections
"""

import sys
import json
import subprocess
from pathlib import Path

def read_pdf_chunks(pdf_path, chunk_lines=500):
    """
    Read PDF file in chunks and return content
    
    Args:
        pdf_path: Path to PDF file
        chunk_lines: Number of lines per chunk
        
    Returns:
        Dictionary with chunks and metadata
    """
    
    pdf_path = Path(pdf_path)
    if not pdf_path.exists():
        return {"error": f"PDF file not found: {pdf_path}"}
    
    # Remove print statements that interfere with JSON output
    # print(f"Processing PDF: {pdf_path}")
    # print(f"File size: {pdf_path.stat().st_size / (1024*1024):.1f} MB")
    
    # Extract text from PDF using pdftotext if available
    try:
        result = subprocess.run(['pdftotext', str(pdf_path), '-'], 
                              capture_output=True, text=True, check=True)
        full_text = result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback: try reading PDF directly (Claude can handle this)
        try:
            with open(pdf_path, 'rb') as f:
                # This won't work for binary PDF, but serves as placeholder
                # In practice, would need a proper PDF library
                return {"error": "pdftotext not available and direct PDF reading not implemented"}
        except Exception as e:
            return {"error": f"Could not read PDF: {e}"}
    
    # Split text into chunks
    lines = full_text.split('\n')
    chunks = []
    
    for i in range(0, len(lines), chunk_lines):
        chunk_text = '\n'.join(lines[i:i+chunk_lines])
        if chunk_text.strip():  # Only include non-empty chunks
            chunks.append({
                "chunk_id": len(chunks),
                "start_line": i,
                "end_line": min(i + chunk_lines - 1, len(lines) - 1),
                "content": chunk_text
            })
    
    return {
        "pdf_path": str(pdf_path),
        "total_lines": len(lines),
        "chunk_lines": chunk_lines,
        "total_chunks": len(chunks),
        "chunks": chunks
    }

def search_pdf_content(content_chunks, search_terms):
    """
    Search for specific terms across PDF content chunks
    
    Args:
        content_chunks: List of content strings from PDF chunks
        search_terms: List of terms to search for
        
    Returns:
        Dictionary with search results
    """
    results = {}
    
    for term in search_terms:
        results[term] = {
            "found": False,
            "locations": [],
            "context": []
        }
        
        for i, chunk in enumerate(content_chunks):
            if term.lower() in chunk.lower():
                results[term]["found"] = True
                results[term]["locations"].append(f"chunk_{i}")
                
                # Extract context around the term
                lines = chunk.split('\n')
                for j, line in enumerate(lines):
                    if term.lower() in line.lower():
                        context_start = max(0, j-2)
                        context_end = min(len(lines), j+3)
                        context = '\n'.join(lines[context_start:context_end])
                        results[term]["context"].append(context)
    
    return results

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python pdf_chunk_reader.py <pdf_path> [chunk_lines]")
        sys.exit(1)
    
    pdf_path = sys.argv[1]
    chunk_lines = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    
    result = read_pdf_chunks(pdf_path, chunk_lines)
    print(json.dumps(result, indent=2))