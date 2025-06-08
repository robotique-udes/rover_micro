#!/usr/bin/env python3
"""
Script to check and fix header guard consistency in C++ header files.
Usage: python3 header_guard_linter.py [--fix] [directory]

Author: AI autogen
"""

from __future__ import annotations

import os
import re
import sys
import argparse
from pathlib import Path
from typing import Optional, List, Set, Final, Literal, Generator, Dict, Tuple
from dataclasses import dataclass
from enum import Enum, auto


class GuardStatus(Enum):
    """Enumeration of possible header guard statuses."""
    OK = auto()
    MISSING = auto()
    MISMATCH = auto()
    INCORRECT = auto()
    ERROR = auto()


class Colors:
    """ANSI color codes for terminal output."""
    RED: Final[str] = '\033[0;31m'
    GREEN: Final[str] = '\033[0;32m'
    YELLOW: Final[str] = '\033[1;33m'
    BLUE: Final[str] = '\033[0;34m'
    NC: Final[str] = '\033[0m'  # No Color


@dataclass(frozen=True)
class GuardCheckResult:
    """Result of a header guard check operation."""
    status: GuardStatus
    current_guard: Optional[str]
    expected_guard: str
    error_message: Optional[str] = None


@dataclass(frozen=True)
class FilePosition:
    """Position information for header guard elements in a file."""
    ifndef_line: Optional[int]
    define_line: Optional[int]
    endif_line: Optional[int]


@dataclass(frozen=True)
class ScanStatistics:
    """Statistics from scanning header files."""
    total_files: int
    ok_files: int
    missing_files: int
    incorrect_files: int
    mismatch_files: int
    error_files: int


@dataclass(frozen=True)
class PathConfig:
    """Configuration for a single path to scan."""
    path: str
    guard_prefix: str
    exclude_dirs: Set[str]
    use_prefix: bool  # Whether to use a prefix at all


class HeaderGuardChecker:
    """Main class for checking and fixing header guards."""
    
    # File extensions to process
    HEADER_EXTENSIONS: Final[Set[str]] = {'.h', '.hpp'}
    
    # Maximum lines to search for header guard elements
    SEARCH_LINES_START: Final[int] = 10
    SEARCH_LINES_END: Final[int] = 10
    
    def __init__(self, base_path: str, guard_prefix: Optional[str] = None, use_prefix: bool = True, exclude_dirs: Optional[List[str]] = None) -> None:
        """
        Initialize the header guard checker.
        
        Args:
            base_path: The base path to use for generating header guards
            guard_prefix: Custom guard prefix. If None, uses last component of base_path
            use_prefix: Whether to use a prefix at all (False for no prefix)
            exclude_dirs: List of directory names to exclude from scanning
        """
        self.base_path: Path = Path(base_path).resolve()
        self.use_prefix: bool = use_prefix
        if use_prefix:
            self.guard_prefix: str = guard_prefix.upper() if guard_prefix else self.base_path.name.upper()
        else:
            self.guard_prefix = ""
        self.exclude_dirs: Set[str] = set(exclude_dirs or ['.platformio'])
    
    def path_to_guard(self, filepath: str) -> str:
        """
        Convert file path to header guard format.
        
        Args:
            filepath: The file path to convert
            
        Returns:
            The expected header guard name
        """
        file_path: Path = Path(filepath).resolve()
        
        if not self.use_prefix:
            # No prefix mode: use relative path from base_path or full relative path
            try:
                relative_path: Path = file_path.relative_to(self.base_path)
            except ValueError:
                # If not under base path, use the filename
                relative_path = Path(file_path.name)
        else:
            # Prefix mode: normal behavior
            try:
                # Get the relative path from the base path
                relative_path = file_path.relative_to(self.base_path)
            except ValueError:
                # If the file is not under the base path, try to find the guard prefix in the path
                relative_path = self._find_relative_path_with_prefix(file_path)
        
        # Convert to header guard format
        guard_path: str = str(relative_path)
        guard_without_prefix: str = guard_path.upper().replace('/', '_').replace('\\', '_').replace('.', '_')
        
        if self.use_prefix and self.guard_prefix:
            return f"{self.guard_prefix}_{guard_without_prefix}"
        else:
            return guard_without_prefix
    
    def _find_relative_path_with_prefix(self, file_path: Path) -> Path:
        """
        Find relative path starting from guard prefix in the file path.
        
        Args:
            file_path: The file path to process
            
        Returns:
            Path starting from the guard prefix or just the filename
        """
        if not self.use_prefix or not self.guard_prefix:
            return Path(file_path.name)
            
        file_parts: Tuple[str, ...] = file_path.parts
        guard_prefix_lower: str = self.guard_prefix.lower()
        
        # Look for the guard prefix in the path components
        for i, part in enumerate(file_parts):
            if part.lower() == guard_prefix_lower:
                return Path(*file_parts[i:])
        
        # Fallback: use just the filename
        return Path(file_path.name)
    
    def _read_file_safe(self, file_path: str) -> Optional[List[str]]:
        """
        Safely read a file and return its lines.
        
        Args:
            file_path: Path to the file to read
            
        Returns:
            List of lines or None if reading failed
        """
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                return f.readlines()
        except (OSError, IOError, UnicodeError) as e:
            print(f"{Colors.RED}Error reading {file_path}: {e}{Colors.NC}")
            return None
    
    def _find_header_guard_positions(self, lines: List[str]) -> FilePosition:
        """
        Find positions of header guard elements in the file.
        
        Args:
            lines: List of file lines
            
        Returns:
            FilePosition with line numbers or None for each element
        """
        ifndef_line: Optional[int] = None
        define_line: Optional[int] = None
        endif_line: Optional[int] = None
        
        # Look for #ifndef in first few lines
        search_end: int = min(len(lines), self.SEARCH_LINES_START)
        for i in range(search_end):
            line: str = lines[i].strip()
            if line.startswith('#ifndef') and ifndef_line is None:
                ifndef_line = i
            elif line.startswith('#define') and define_line is None:
                define_line = i
        
        # Look for #endif in last few lines
        search_start: int = max(0, len(lines) - self.SEARCH_LINES_END)
        for i in range(len(lines) - 1, search_start - 1, -1):
            if lines[i].strip().startswith('#endif'):
                endif_line = i
                break
        
        return FilePosition(ifndef_line, define_line, endif_line)
    
    def _extract_guard_name(self, line: str, directive: Literal['#ifndef', '#define']) -> Optional[str]:
        """
        Extract guard name from a preprocessor directive line.
        
        Args:
            line: The line containing the directive
            directive: The directive type ('#ifndef' or '#define')
            
        Returns:
            The guard name or None if not found
        """
        pattern: str = rf'^\s*{re.escape(directive)}\s+([A-Za-z_][A-Za-z0-9_]*)'
        match: Optional[re.Match[str]] = re.match(pattern, line)
        return match.group(1) if match else None
    
    def check_header_guard(self, file_path: str) -> GuardCheckResult:
        """
        Check if file has proper header guards.
        
        Args:
            file_path: Path to the file to check
            
        Returns:
            GuardCheckResult with status and guard information
        """
        expected_guard: str = self.path_to_guard(file_path)
        
        lines: Optional[List[str]] = self._read_file_safe(file_path)
        if lines is None:
            return GuardCheckResult(
                GuardStatus.ERROR, 
                None, 
                expected_guard, 
                "Failed to read file"
            )
        
        if not lines:
            return GuardCheckResult(GuardStatus.MISSING, None, expected_guard)
        
        positions: FilePosition = self._find_header_guard_positions(lines)
        
        # Check if all parts are present
        if not self._has_complete_guard(positions):
            return GuardCheckResult(GuardStatus.MISSING, None, expected_guard)
        
        # Extract guard names
        ifndef_guard: Optional[str] = self._extract_guard_name(lines[positions.ifndef_line], '#ifndef')
        define_guard: Optional[str] = self._extract_guard_name(lines[positions.define_line], '#define')
        
        if ifndef_guard is None or define_guard is None:
            return GuardCheckResult(GuardStatus.MISSING, None, expected_guard)
        
        # Check if guards match each other
        if ifndef_guard != define_guard:
            return GuardCheckResult(GuardStatus.MISMATCH, ifndef_guard, expected_guard)
        
        # Check if guard matches expected format
        if ifndef_guard != expected_guard:
            return GuardCheckResult(GuardStatus.INCORRECT, ifndef_guard, expected_guard)
        
        return GuardCheckResult(GuardStatus.OK, ifndef_guard, expected_guard)
    
    def _has_complete_guard(self, positions: FilePosition) -> bool:
        """Check if all guard positions are present."""
        return all([
            positions.ifndef_line is not None,
            positions.define_line is not None,
            positions.endif_line is not None
        ])
    
    def _write_file_safe(self, file_path: str, lines: List[str]) -> bool:
        """
        Safely write lines to a file.
        
        Args:
            file_path: Path to the file to write
            lines: List of lines to write
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.writelines(lines)
            return True
        except (OSError, IOError, UnicodeError) as e:
            print(f"{Colors.RED}Error writing {file_path}: {e}{Colors.NC}")
            return False
    
    def fix_header_guard(self, file_path: str) -> bool:
        """
        Fix header guard in a file.
        
        Args:
            file_path: Path to the file to fix
            
        Returns:
            True if successful, False otherwise
        """
        expected_guard: str = self.path_to_guard(file_path)
        
        lines: Optional[List[str]] = self._read_file_safe(file_path)
        if lines is None:
            return False
        
        positions: FilePosition = self._find_header_guard_positions(lines)
        
        if self._has_complete_guard(positions):
            # Replace existing header guards
            lines = self._replace_existing_guards(lines, positions, expected_guard)
        else:
            # Add header guards to file without them
            lines = self._add_new_guards(lines, expected_guard)
        
        return self._write_file_safe(file_path, lines)
    
    def _replace_existing_guards(self, lines: List[str], positions: FilePosition, expected_guard: str) -> List[str]:
        """Replace existing header guard lines."""
        lines[positions.ifndef_line] = f'#ifndef {expected_guard}\n'
        lines[positions.define_line] = f'#define {expected_guard}\n'
        lines[positions.endif_line] = f'#endif  // {expected_guard}\n'
        return lines
    
    def _add_new_guards(self, lines: List[str], expected_guard: str) -> List[str]:
        """Add header guards to a file that doesn't have them."""
        new_lines: List[str] = [
            f'#ifndef {expected_guard}\n',
            f'#define {expected_guard}\n',
            '\n'
        ]
        new_lines.extend(lines)
        new_lines.extend([
            '\n',
            f'#endif  // {expected_guard}\n'
        ])
        return new_lines
    
    def _should_skip_file(self, file_path: Path) -> bool:
        """
        Check if a file should be skipped based on exclude directories.
        
        Args:
            file_path: Path to check
            
        Returns:
            True if file should be skipped, False otherwise
        """
        file_str: str = str(file_path)
        return any(exclude_dir in file_str for exclude_dir in self.exclude_dirs)
    
    def _find_header_files(self, target_dir: str) -> Generator[Path, None, None]:
        """
        Find all header files in the target directory.
        
        Args:
            target_dir: Directory to search
            
        Yields:
            Path objects for header files
        """
        target_path: Path = Path(target_dir)
        
        for extension in ['*.h', '*.hpp']:
            for file_path in target_path.rglob(extension):
                if not self._should_skip_file(file_path):
                    yield file_path
    
    def _get_display_path(self, file_path: str) -> str:
        """
        Get a display path that shows the guard prefix context.
        
        Args:
            file_path: The full file path
            
        Returns:
            A display path starting from the guard prefix
        """
        file_path_obj: Path = Path(file_path)
        
        if not self.use_prefix or not self.guard_prefix:
            # No prefix mode: show relative to base path
            try:
                return str(file_path_obj.relative_to(self.base_path))
            except ValueError:
                return file_path_obj.name
        
        file_parts: Tuple[str, ...] = file_path_obj.parts
        guard_prefix_lower: str = self.guard_prefix.lower()
        
        # Look for the guard prefix in the path components
        for i, part in enumerate(file_parts):
            if part.lower() == guard_prefix_lower:
                relevant_parts: Tuple[str, ...] = file_parts[i:]
                return str(Path(*relevant_parts))
        
        # Fallback: show relative to base path or just filename
        try:
            return str(file_path_obj.relative_to(self.base_path))
        except ValueError:
            return file_path_obj.name
    
    def _print_file_status(self, file_path: str, result: GuardCheckResult) -> None:
        """
        Print the status of a single file check.
        
        Args:
            file_path: Path to the file
            result: Result of the header guard check
        """
        display_path: str = self._get_display_path(file_path)
        
        status_messages: Dict[GuardStatus, str] = {
            GuardStatus.OK: f"{Colors.GREEN}[OK]{Colors.NC} {display_path}",
            GuardStatus.MISSING: f"{Colors.RED}[MISS]{Colors.NC} {display_path} {Colors.YELLOW}(missing header guards){Colors.NC}",
            GuardStatus.MISMATCH: f"{Colors.RED}[MISM]{Colors.NC} {display_path} {Colors.YELLOW}(#ifndef and #define don't match){Colors.NC}",
            GuardStatus.INCORRECT: f"{Colors.RED}[INCR]{Colors.NC} {display_path} {Colors.YELLOW}(expected: {result.expected_guard}, found: {result.current_guard}){Colors.NC}",
            GuardStatus.ERROR: f"{Colors.RED}[ERR]{Colors.NC} {display_path} {Colors.YELLOW}({result.error_message or 'Unknown error'}){Colors.NC}"
        }
        
        print(status_messages[result.status])
    
    def _print_summary(self, stats: ScanStatistics, fix_mode: bool, original_command: str) -> None:
        """
        Print scan summary statistics.
        
        Args:
            stats: Scan statistics
            fix_mode: Whether fix mode was enabled
            original_command: The original command line used
        """
        print()
        print(f"{Colors.BLUE}=== SUMMARY ==={Colors.NC}")
        print(f"Total files scanned: {stats.total_files}")
        print(f"{Colors.GREEN}OK: {stats.ok_files}{Colors.NC}")
        print(f"{Colors.RED}Missing header guards: {stats.missing_files}{Colors.NC}")
        print(f"{Colors.RED}Mismatched guards: {stats.mismatch_files}{Colors.NC}")
        print(f"{Colors.RED}Incorrect guards: {stats.incorrect_files}{Colors.NC}")
        
        if stats.error_files > 0:
            print(f"{Colors.RED}Errors: {stats.error_files}{Colors.NC}")
        
        if fix_mode:
            self._print_fix_summary(stats)
        else:
            self._print_fix_suggestion(stats, original_command)
    
    def _print_fix_summary(self, stats: ScanStatistics) -> None:
        """Print summary for fix mode."""
        fixed_files: int = stats.missing_files + stats.mismatch_files + stats.incorrect_files
        if fixed_files > 0:
            print(f"{Colors.GREEN}Fixed {fixed_files} files{Colors.NC}")
    
    def _print_fix_suggestion(self, stats: ScanStatistics, original_command: str) -> None:
        """Print suggestion to run with --fix flag."""
        problem_files: int = stats.missing_files + stats.mismatch_files + stats.incorrect_files
        if problem_files > 0:
            print()
            fix_command: str = self._generate_fix_command(original_command)
            print(f"{Colors.YELLOW}Run with --fix to automatically correct these issues:{Colors.NC}")
            print(f"  {fix_command}")
    
    def _generate_fix_command(self, original_command: str) -> str:
        """Generate fix command by adding --fix to the original command."""
        if '--fix' not in original_command:
            parts: List[str] = original_command.split()
            if parts:
                return f"{parts[0]} {parts[1]} --fix {' '.join(parts[2:])}"
        return original_command
    
    def scan_directory(self, target_dir: str, fix_mode: bool = False, original_command: str = "") -> ScanStatistics:
        """
        Scan directory for header files and check guards.
        
        Args:
            target_dir: Directory to scan
            fix_mode: Whether to automatically fix issues
            original_command: The original command line used
            
        Returns:
            ScanStatistics with the results
        """
        self._print_scan_header(target_dir)
        
        stats_counters: Dict[str, int] = {
            'total_files': 0,
            'ok_files': 0,
            'missing_files': 0,
            'incorrect_files': 0,
            'mismatch_files': 0,
            'error_files': 0
        }
        
        for file_path in self._find_header_files(target_dir):
            stats_counters['total_files'] += 1
            file_str: str = str(file_path)
            
            result: GuardCheckResult = self.check_header_guard(file_str)
            self._print_file_status(file_str, result)
            
            # Update counters and fix if needed
            self._process_file_result(result, file_str, fix_mode, stats_counters)
        
        stats: ScanStatistics = ScanStatistics(**stats_counters)
        self._print_summary(stats, fix_mode, original_command)
        
        return stats
    
    def _print_scan_header(self, target_dir: str) -> None:
        """Print scan header information."""
        print(f"{Colors.BLUE}Scanning directory: {target_dir}{Colors.NC}")
        print(f"{Colors.BLUE}Base path: {self.base_path}{Colors.NC}")
        
        if self.use_prefix:
            prefix_display: str = self.guard_prefix if self.guard_prefix else "(auto-detected)"
            print(f"{Colors.BLUE}Header guard prefix: {prefix_display}{Colors.NC}")
        else:
            print(f"{Colors.BLUE}Header guard prefix: (none - using relative paths){Colors.NC}")
            
        print(f"{Colors.BLUE}Excluding: {', '.join(sorted(self.exclude_dirs))}{Colors.NC}")
        print()
    
    def _process_file_result(self, result: GuardCheckResult, file_str: str, fix_mode: bool, counters: Dict[str, int]) -> None:
        """Process the result of checking a single file."""
        from typing import Callable
        
        status_handlers: Dict[GuardStatus, Callable[[], None]] = {
            GuardStatus.OK: lambda: self._increment_counter(counters, 'ok_files'),
            GuardStatus.MISSING: lambda: self._handle_fixable_status(counters, 'missing_files', file_str, fix_mode),
            GuardStatus.MISMATCH: lambda: self._handle_fixable_status(counters, 'mismatch_files', file_str, fix_mode),
            GuardStatus.INCORRECT: lambda: self._handle_fixable_status(counters, 'incorrect_files', file_str, fix_mode),
            GuardStatus.ERROR: lambda: self._increment_counter(counters, 'error_files')
        }
        
        status_handlers[result.status]()
    
    def _increment_counter(self, counters: Dict[str, int], counter_name: str) -> None:
        """Increment a counter in the counters dictionary."""
        counters[counter_name] += 1
    
    def _handle_fixable_status(self, counters: Dict[str, int], counter_name: str, file_str: str, fix_mode: bool) -> None:
        """Handle a status that can be fixed."""
        self._increment_counter(counters, counter_name)
        if fix_mode and self.fix_header_guard(file_str):
            print(f"{Colors.GREEN}Fixed{Colors.NC}: {self._get_display_path(file_str)}")


def _parse_path_with_prefix(path_spec: str) -> Tuple[str, Optional[str], bool]:
    """
    Parse a path specification that may include a custom prefix.
    
    Formats supported:
    - "path" -> (path, None, True)
    - "path:prefix" -> (path, prefix, True)
    - "path:''" or "path:" -> (path, "", False)  # No prefix
    - "prefix=path" -> (path, prefix, True)
    - "=path" -> (path, "", False)  # No prefix
    
    Args:
        path_spec: Path specification string
        
    Returns:
        Tuple of (path, prefix, use_prefix) where:
        - prefix is None if not specified (auto-detect)
        - prefix is "" and use_prefix is False for no prefix
        - use_prefix indicates whether to use a prefix at all
    """
    # Check for colon syntax: path:prefix
    if ':' in path_spec and not (len(path_spec) > 1 and path_spec[1] == ':'):  # Avoid Windows drive letters
        parts: List[str] = path_spec.split(':', 1)
        if len(parts) == 2:
            path, prefix = parts[0], parts[1]
            if prefix == '' or prefix == "''":  # Empty prefix means no prefix
                return path, "", False
            return path, prefix, True
    
    # Check for equals syntax: prefix=path
    if '=' in path_spec:
        parts = path_spec.split('=', 1)
        if len(parts) == 2:
            prefix, path = parts[0], parts[1]
            if prefix == '':  # Empty prefix means no prefix
                return path, "", False
            return path, prefix, True
    
    # Default: just a path
    return path_spec, None, True


def create_argument_parser() -> argparse.ArgumentParser:
    """Create and configure the argument parser."""
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description='Check and fix header guard consistency in C++ header files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s ../lib/rover_lib2/src/rover_lib2
    # Use 'rover_lib2' as guard prefix (last component of path)
    # File: .../rover_lib2/communication/SPI/SPI_bus.hpp
    # Guard: ROVER_LIB2_COMMUNICATION_SPI_SPI_BUS_HPP
  
  %(prog)s --fix /home/user/project/lib/rover_lib2/src/rover_lib2  
    # Fix all issues using 'rover_lib2' as guard prefix
    
  %(prog)s lib/my_library:MYLIB ../other_lib/src/other_lib:OTHERLIB
    # Use custom guard prefixes with colon syntax
    # File: lib/my_library/utils/helper.h
    # Guard: MYLIB_UTILS_HELPER_H
    
  %(prog)s lib/my_library:'' src/common:
    # No prefix - use relative paths only
    # File: lib/my_library/utils/helper.h
    # Guard: UTILS_HELPER_H
    
  %(prog)s MYPREFIX=lib/my_library =src/common
    # Use custom guard prefixes with equals syntax
    # Second path uses no prefix (empty before =)
    
  %(prog)s --fix lib/rover_lib2:ROVER lib/rover_can2: lib/rover_helpers:HELPERS
    # Mix prefixed and non-prefixed paths

Path Prefix Formats:
  path                    # Auto-detect prefix from last path component
  path:prefix             # Use custom prefix (colon syntax)
  path:'' or path:        # No prefix - use relative paths only
  prefix=path             # Use custom prefix (equals syntax)  
  =path                   # No prefix - use relative paths only

Guard Generation:
  - With prefix: PREFIX_RELATIVE_PATH_FILE_EXT
  - No prefix: RELATIVE_PATH_FILE_EXT
  - Relative paths are calculated from the base path or from first
    occurrence of prefix in the file path (case-insensitive)
        """.strip()
    )
    
    parser.add_argument(
        '--fix', 
        action='store_true', 
        help='Automatically fix incorrect header guards'
    )
    
    parser.add_argument(
        'path_specs', 
        nargs='+',
        help='Path specifications with optional custom prefixes (path, path:prefix, path:\'\', prefix=path, or =path)'
    )
    
    parser.add_argument(
        '--exclude', 
        action='append', 
        help='Additional directories to exclude (can be used multiple times)'
    )
    
    return parser


def main() -> int:
    """
    Main entry point.
    
    Returns:
        Exit code (0 for success, 1 for failure)
    """
    parser: argparse.ArgumentParser = create_argument_parser()
    args: argparse.Namespace = parser.parse_args()
    
    # Parse path specifications
    path_configs: List[PathConfig] = []
    for path_spec in args.path_specs:
        path, custom_prefix, use_prefix = _parse_path_with_prefix(path_spec)
        
        if use_prefix:
            guard_prefix = custom_prefix if custom_prefix is not None else Path(path).name.upper()
        else:
            guard_prefix = ""
            
        path_configs.append(PathConfig(
            path=path,
            guard_prefix=guard_prefix,
            exclude_dirs=set(args.exclude or ['.platformio']),
            use_prefix=use_prefix
        ))
    
    # Reconstruct the original command line for the fix suggestion
    original_command: str = f"python3 {sys.argv[0]} {' '.join(sys.argv[1:])}"
    
    # Validate all paths exist
    if not _validate_path_configs(path_configs):
        return 1
    
    if args.fix:
        print(f"{Colors.YELLOW}Running in fix mode - files will be modified{Colors.NC}")
        print()
    
    all_success: bool = True
    total_stats: ScanStatistics = ScanStatistics(0, 0, 0, 0, 0, 0)
    
    for i, config in enumerate(path_configs):
        abs_base_path: str = os.path.abspath(config.path)
        
        _print_path_header(i, len(path_configs), config.path, config.guard_prefix, config.use_prefix)
        
        checker: HeaderGuardChecker = HeaderGuardChecker(
            base_path=abs_base_path,
            guard_prefix=config.guard_prefix if config.use_prefix else None,
            use_prefix=config.use_prefix,
            exclude_dirs=list(config.exclude_dirs)
        )
        scan_dir: str = _get_scan_directory(abs_base_path)
        
        stats: ScanStatistics = checker.scan_directory(scan_dir, args.fix, original_command)
        total_stats = _accumulate_stats(total_stats, stats)
        
        # Check if this path had issues
        if _has_unfixed_problems(stats, args.fix):
            all_success = False
        
        _print_path_separator(i, len(path_configs))
    
    # Print overall summary if multiple paths were processed
    if len(path_configs) > 1:
        _print_overall_summary(total_stats, args.fix)
    
    return _get_exit_code(all_success)


def _validate_path_configs(path_configs: List[PathConfig]) -> bool:
    """Validate that all paths in configurations exist."""
    for config in path_configs:
        if not os.path.exists(config.path):
            print(f"{Colors.RED}Error: Path '{config.path}' does not exist{Colors.NC}")
            return False
    return True


def _print_path_header(index: int, total_paths: int, base_path: str, guard_prefix: str, use_prefix: bool) -> None:
    """Print header for processing a specific path."""
    if total_paths > 1:
        print(f"{Colors.BLUE}{'='*60}{Colors.NC}")
        print(f"{Colors.BLUE}Processing path {index+1}/{total_paths}: {base_path}{Colors.NC}")
        
        if use_prefix:
            print(f"{Colors.BLUE}Guard prefix: {guard_prefix}{Colors.NC}")
        else:
            print(f"{Colors.BLUE}Guard prefix: (none - using relative paths){Colors.NC}")
            
        print(f"{Colors.BLUE}{'='*60}{Colors.NC}")


def _get_scan_directory(abs_base_path: str) -> str:
    """Get the directory to scan based on the base path."""
    if os.path.isfile(abs_base_path):
        scan_dir: str = os.path.dirname(abs_base_path)
        print(f"{Colors.YELLOW}Note: Base path is a file, scanning its parent directory: {scan_dir}{Colors.NC}")
        return scan_dir
    return abs_base_path


def _accumulate_stats(total_stats: ScanStatistics, stats: ScanStatistics) -> ScanStatistics:
    """Accumulate statistics from multiple scans."""
    return ScanStatistics(
        total_files=total_stats.total_files + stats.total_files,
        ok_files=total_stats.ok_files + stats.ok_files,
        missing_files=total_stats.missing_files + stats.missing_files,
        incorrect_files=total_stats.incorrect_files + stats.incorrect_files,
        mismatch_files=total_stats.mismatch_files + stats.mismatch_files,
        error_files=total_stats.error_files + stats.error_files
    )


def _has_unfixed_problems(stats: ScanStatistics, fix_mode: bool) -> bool:
    """Check if there are unfixed problems."""
    problem_files: int = stats.missing_files + stats.mismatch_files + stats.incorrect_files
    return problem_files > 0 and not fix_mode


def _print_path_separator(index: int, total_paths: int) -> None:
    """Print separator between multiple paths."""
    if total_paths > 1 and index < total_paths - 1:
        print()


def _print_overall_summary(total_stats: ScanStatistics, fix_mode: bool) -> None:
    """Print overall summary for multiple paths."""
    print()
    print(f"{Colors.BLUE}=== OVERALL SUMMARY ==={Colors.NC}")
    print(f"Total files scanned: {total_stats.total_files}")
    print(f"{Colors.GREEN}OK: {total_stats.ok_files}{Colors.NC}")
    print(f"{Colors.RED}Missing header guards: {total_stats.missing_files}{Colors.NC}")
    print(f"{Colors.RED}Mismatched guards: {total_stats.mismatch_files}{Colors.NC}")
    print(f"{Colors.RED}Incorrect guards: {total_stats.incorrect_files}{Colors.NC}")
    
    if total_stats.error_files > 0:
        print(f"{Colors.RED}Errors: {total_stats.error_files}{Colors.NC}")
    
    if fix_mode:
        fixed_files: int = total_stats.missing_files + total_stats.mismatch_files + total_stats.incorrect_files
        if fixed_files > 0:
            print(f"{Colors.GREEN}Total fixed files: {fixed_files}{Colors.NC}")


def _get_exit_code(all_success: bool) -> int:
    """Get the appropriate exit code."""
    if all_success:
        print(f"{Colors.GREEN}All header guards are properly formatted!{Colors.NC}")
        return 0
    else:
        print(f"{Colors.RED}Some header guards need attention{Colors.NC}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
