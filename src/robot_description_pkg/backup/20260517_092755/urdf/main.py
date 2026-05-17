import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import os
import shutil
import threading
from datetime import datetime

class FileTransferApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Safe Drive Transfer - C: to D:")
        self.root.geometry("900x700")
        self.root.configure(bg='#1a1a2e')
        
        # Windows system paths to avoid
        self.windows_paths = [
            'windows', 'program files', 'program files (x86)', 'programdata',
            'users', 'inetpub', 'recovery', 'system volume information',
            'config.msi', 'perflogs', 'drivers', 'syswow64', 'winsxs'
        ]
        
        self.source_path = tk.StringVar(value="C:\\")
        self.dest_path = tk.StringVar(value="D:\\")
        self.status_var = tk.StringVar(value="Ready")
        self.progress_var = tk.DoubleVar(value=0)
        self.total_files = 0
        self.transferred_files = 0
        
        self.setup_styles()
        self.create_widgets()
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # Custom colors
        bg_color = '#1a1a2e'
        accent_color = '#16213e'
        highlight = '#e94560'
        text_color = '#eaeaea'
        
        style.configure('Custom.TFrame', background=bg_color)
        style.configure('Custom.TLabel', background=bg_color, foreground=text_color, font=('Segoe UI', 10))
        style.configure('Header.TLabel', background=bg_color, foreground=highlight, font=('Segoe UI', 16, 'bold'))
        style.configure('Custom.TButton', background=highlight, foreground='white', font=('Segoe UI', 10, 'bold'))
        style.configure('Custom.TCheckbutton', background=bg_color, foreground=text_color)
        style.configure('Custom.Horizontal.TProgressbar', background=highlight, troughcolor=accent_color)
        
    def create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root, style='Custom.TFrame')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Header
        header = ttk.Label(main_frame, text="🚀 Safe File Transfer Utility", style='Header.TLabel')
        header.pack(pady=(0, 20))
        
        # Warning banner
        warning_frame = tk.Frame(main_frame, bg='#16213e', bd=2, relief=tk.RIDGE)
        warning_frame.pack(fill=tk.X, pady=(0, 15))
        
        warning_text = """⚠️  SAFETY NOTICE: This tool automatically skips Windows system files and protected directories.
    It will NOT transfer: Windows, Program Files, Users, System folders, or hidden system files."""
        warning_label = tk.Label(warning_frame, text=warning_text, bg='#16213e', fg='#ffd700', 
                                font=('Segoe UI', 9), justify=tk.LEFT, wraplength=800)
        warning_label.pack(padx=10, pady=10)
        
        # Path selection frame
        path_frame = tk.Frame(main_frame, bg='#1a1a2e')
        path_frame.pack(fill=tk.X, pady=10)
        
        # Source selection
        tk.Label(path_frame, text="Source (C:)", bg='#1a1a2e', fg='#eaeaea', 
                font=('Segoe UI', 11, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=5)
        
        source_entry = tk.Entry(path_frame, textvariable=self.source_path, width=50, 
                               font=('Consolas', 10), bg='#16213e', fg='#eaeaea', 
                               insertbackground='#eaeaea', relief=tk.FLAT)
        source_entry.grid(row=0, column=1, padx=10, pady=5, ipady=5)
        
        tk.Button(path_frame, text="Browse", command=self.browse_source, bg='#e94560', 
                 fg='white', font=('Segoe UI', 9, 'bold'), relief=tk.FLAT, cursor='hand2',
                 activebackground='#ff6b6b').grid(row=0, column=2, padx=5)
        
        # Destination selection
        tk.Label(path_frame, text="Destination (D:)", bg='#1a1a2e', fg='#eaeaea', 
                font=('Segoe UI', 11, 'bold')).grid(row=1, column=0, sticky=tk.W, pady=5)
        
        dest_entry = tk.Entry(path_frame, textvariable=self.dest_path, width=50, 
                             font=('Consolas', 10), bg='#16213e', fg='#eaeaea', 
                             insertbackground='#eaeaea', relief=tk.FLAT)
        dest_entry.grid(row=1, column=1, padx=10, pady=5, ipady=5)
        
        tk.Button(path_frame, text="Browse", command=self.browse_dest, bg='#e94560', 
                 fg='white', font=('Segoe UI', 9, 'bold'), relief=tk.FLAT, cursor='hand2',
                 activebackground='#ff6b6b').grid(row=1, column=2, padx=5)
        
        # Options frame
        options_frame = tk.LabelFrame(main_frame, text=" Transfer Options ", bg='#1a1a2e', 
                                     fg='#e94560', font=('Segoe UI', 11, 'bold'), bd=2)
        options_frame.pack(fill=tk.X, pady=15)
        
        # File type filters
        self.transfer_images = tk.BooleanVar(value=True)
        self.transfer_videos = tk.BooleanVar(value=True)
        self.transfer_docs = tk.BooleanVar(value=True)
        self.transfer_audio = tk.BooleanVar(value=True)
        self.transfer_others = tk.BooleanVar(value=False)
        
        filters = [
            ("Images (jpg, png, gif, bmp, webp)", self.transfer_images),
            ("Videos (mp4, avi, mkv, mov, wmv)", self.transfer_videos),
            ("Documents (pdf, doc, txt, xls, ppt)", self.transfer_docs),
            ("Audio (mp3, wav, flac, aac)", self.transfer_audio),
            ("Other files", self.transfer_others)
        ]
        
        for i, (text, var) in enumerate(filters):
            cb = tk.Checkbutton(options_frame, text=text, variable=var, bg='#1a1a2e', 
                               fg='#eaeaea', selectcolor='#16213e', activebackground='#1a1a2e',
                               activeforeground='#eaeaea', font=('Segoe UI', 10))
            cb.grid(row=i//3, column=i%3, sticky=tk.W, padx=15, pady=5)
        
        # Additional options
        self.preserve_structure = tk.BooleanVar(value=True)
        self.skip_duplicates = tk.BooleanVar(value=True)
        self.verify_transfer = tk.BooleanVar(value=True)
        
        extra_opts = tk.Frame(options_frame, bg='#1a1a2e')
        extra_opts.grid(row=2, column=0, columnspan=3, pady=10)
        
        tk.Checkbutton(extra_opts, text="Preserve folder structure", variable=self.preserve_structure,
                      bg='#1a1a2e', fg='#eaeaea', selectcolor='#16213e').pack(side=tk.LEFT, padx=10)
        tk.Checkbutton(extra_opts, text="Skip duplicates", variable=self.skip_duplicates,
                      bg='#1a1a2e', fg='#eaeaea', selectcolor='#16213e').pack(side=tk.LEFT, padx=10)
        tk.Checkbutton(extra_opts, text="Verify after transfer", variable=self.verify_transfer,
                      bg='#1a1a2e', fg='#eaeaea', selectcolor='#16213e').pack(side=tk.LEFT, padx=10)
        
        # Action buttons
        btn_frame = tk.Frame(main_frame, bg='#1a1a2e')
        btn_frame.pack(pady=15)
        
        self.analyze_btn = tk.Button(btn_frame, text="🔍 Analyze Files", command=self.analyze_files,
                                    bg='#0f3460', fg='white', font=('Segoe UI', 11, 'bold'),
                                    relief=tk.FLAT, cursor='hand2', padx=20, pady=10,
                                    activebackground='#1a4a7a')
        self.analyze_btn.pack(side=tk.LEFT, padx=10)
        
        self.transfer_btn = tk.Button(btn_frame, text="🚀 Start Transfer", command=self.start_transfer,
                                     bg='#e94560', fg='white', font=('Segoe UI', 11, 'bold'),
                                     relief=tk.FLAT, cursor='hand2', padx=20, pady=10,
                                     activebackground='#ff6b6b', state=tk.DISABLED)
        self.transfer_btn.pack(side=tk.LEFT, padx=10)
        
        # Progress section
        progress_frame = tk.Frame(main_frame, bg='#1a1a2e')
        progress_frame.pack(fill=tk.X, pady=10)
        
        self.progress_bar = ttk.Progressbar(progress_frame, variable=self.progress_var, 
                                          maximum=100, length=800, mode='determinate',
                                          style='Custom.Horizontal.TProgressbar')
        self.progress_bar.pack(fill=tk.X)
        
        self.status_label = tk.Label(progress_frame, textvariable=self.status_var, 
                                    bg='#1a1a2e', fg='#00d9ff', font=('Segoe UI', 10))
        self.status_label.pack(pady=5)
        
        # Log area
        log_frame = tk.LabelFrame(main_frame, text=" Transfer Log ", bg='#1a1a2e', 
                                 fg='#e94560', font=('Segoe UI', 11, 'bold'), bd=2)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, 
                                                 font=('Consolas', 9), bg='#16213e', 
                                                 fg='#eaeaea', insertbackground='#eaeaea',
                                                 relief=tk.FLAT, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Statistics
        self.stats_label = tk.Label(main_frame, text="Files found: 0 | Size: 0 MB | Protected: 0", 
                                   bg='#1a1a2e', fg='#eaeaea', font=('Segoe UI', 10))
        self.stats_label.pack(pady=5)
        
    def browse_source(self):
        folder = filedialog.askdirectory(initialdir=self.source_path.get())
        if folder:
            self.source_path.set(folder)
            
    def browse_dest(self):
        folder = filedialog.askdirectory(initialdir=self.dest_path.get())
        if folder:
            self.dest_path.set(folder)
    
    def log(self, message, tag=""):
        self.log_text.configure(state=tk.NORMAL)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)
        
    def is_windows_path(self, path):
        """Check if path is a Windows system directory"""
        path_lower = path.lower()
        for sys_path in self.windows_paths:
            if sys_path in path_lower:
                return True
        return False
    
    def get_allowed_extensions(self):
        """Get list of allowed extensions based on user selection"""
        extensions = []
        if self.transfer_images.get():
            extensions.extend(['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.webp', '.tiff', '.ico', '.svg'])
        if self.transfer_videos.get():
            extensions.extend(['.mp4', '.avi', '.mkv', '.mov', '.wmv', '.flv', '.webm', '.m4v', '.mpg', '.mpeg'])
        if self.transfer_docs.get():
            extensions.extend(['.pdf', '.doc', '.docx', '.txt', '.rtf', '.xls', '.xlsx', '.ppt', '.pptx', '.csv'])
        if self.transfer_audio.get():
            extensions.extend(['.mp3', '.wav', '.flac', '.aac', '.ogg', '.m4a', '.wma'])
        return extensions
    
    def analyze_files(self):
        """Analyze source directory and count transferable files"""
        source = self.source_path.get()
        
        if not os.path.exists(source):
            messagebox.showerror("Error", f"Source path does not exist: {source}")
            return
            
        if self.is_windows_path(source):
            messagebox.showwarning("Warning", "This appears to be a Windows system directory. Transfer blocked for safety.")
            return
        
        self.log("🔍 Starting analysis...", "info")
        self.analyze_btn.configure(state=tk.DISABLED)
        
        def analyze_thread():
            try:
                total_size = 0
                file_count = 0
                protected_count = 0
                allowed_exts = self.get_allowed_extensions()
                
                for root_dir, dirs, files in os.walk(source):
                    # Remove protected directories from traversal
                    dirs[:] = [d for d in dirs if not self.is_windows_path(os.path.join(root_dir, d))]
                    
                    if self.is_windows_path(root_dir):
                        protected_count += len(files)
                        continue
                    
                    for file in files:
                        file_path = os.path.join(root_dir, file)
                        ext = os.path.splitext(file.lower())[1]
                        
                        if ext in allowed_exts or (self.transfer_others.get() and ext not in ['.sys', '.dll', '.exe', '.msi', '.tmp']):
                            try:
                                size = os.path.getsize(file_path)
                                total_size += size
                                file_count += 1
                            except:
                                pass
                
                self.total_files = file_count
                
                # Update UI
                self.root.after(0, lambda: self.update_analysis_results(file_count, total_size, protected_count))
                
            except Exception as e:
                self.root.after(0, lambda: self.log(f"❌ Analysis error: {str(e)}", "error"))
                self.root.after(0, lambda: self.analyze_btn.configure(state=tk.NORMAL))
        
        threading.Thread(target=analyze_thread, daemon=True).start()
    
    def update_analysis_results(self, count, size, protected):
        size_mb = size / (1024 * 1024)
        self.stats_label.configure(text=f"Files found: {count} | Size: {size_mb:.2f} MB | Protected: {protected}")
        self.log(f"✅ Analysis complete: {count} files ({size_mb:.2f} MB) ready to transfer", "success")
        self.log(f"🛡️  Protected files skipped: {protected}", "info")
        
        if count > 0:
            self.transfer_btn.configure(state=tk.NORMAL)
        else:
            self.log("⚠️  No transferable files found with current filters", "warning")
            
        self.analyze_btn.configure(state=tk.NORMAL)
    
    def start_transfer(self):
        """Begin file transfer process"""
        source = self.source_path.get()
        dest = self.dest_path.get()
        
        if not os.path.exists(dest):
            messagebox.showerror("Error", f"Destination does not exist: {dest}")
            return
        
        if source.lower() == dest.lower():
            messagebox.showerror("Error", "Source and destination cannot be the same!")
            return
        
        confirm = messagebox.askyesno("Confirm Transfer", 
                                    f"Transfer {self.total_files} files from\n{source}\nto\n{dest}?")
        if not confirm:
            return
        
        self.transfer_btn.configure(state=tk.DISABLED)
        self.analyze_btn.configure(state=tk.DISABLED)
        self.progress_var.set(0)
        self.transferred_files = 0
        
        def transfer_thread():
            try:
                self.log("🚀 Starting transfer...", "info")
                allowed_exts = self.get_allowed_extensions()
                processed = 0
                
                for root_dir, dirs, files in os.walk(source):
                    # Skip Windows directories
                    dirs[:] = [d for d in dirs if not self.is_windows_path(os.path.join(root_dir, d))]
                    
                    if self.is_windows_path(root_dir):
                        continue
                    
                    # Calculate relative path for structure preservation
                    rel_path = os.path.relpath(root_dir, source)
                    dest_dir = os.path.join(dest, rel_path) if self.preserve_structure.get() else dest
                    
                    for file in files:
                        file_path = os.path.join(root_dir, file)
                        ext = os.path.splitext(file.lower())[1]
                        
                        if ext in allowed_exts or (self.transfer_others.get() and ext not in ['.sys', '.dll', '.exe', '.msi', '.tmp']):
                            try:
                                # Create destination directory if needed
                                if not os.path.exists(dest_dir):
                                    os.makedirs(dest_dir, exist_ok=True)
                                
                                dest_file = os.path.join(dest_dir, file)
                                
                                # Handle duplicates
                                if os.path.exists(dest_file) and self.skip_duplicates.get():
                                    self.root.after(0, lambda f=file: self.log(f"⏭️  Skipped (duplicate): {f}", "info"))
                                    processed += 1
                                    continue
                                
                                # Handle name conflicts
                                counter = 1
                                original_dest = dest_file
                                while os.path.exists(dest_file) and not self.skip_duplicates.get():
                                    name, extension = os.path.splitext(original_dest)
                                    dest_file = f"{name}_{counter}{extension}"
                                    counter += 1
                                
                                # Copy file
                                shutil.copy2(file_path, dest_file)
                                
                                # Verify if requested
                                if self.verify_transfer.get():
                                    if os.path.getsize(file_path) == os.path.getsize(dest_file):
                                        self.root.after(0, lambda f=file: self.log(f"✅ Transferred: {f}", "success"))
                                    else:
                                        self.root.after(0, lambda f=file: self.log(f"⚠️  Size mismatch: {f}", "warning"))
                                else:
                                    self.root.after(0, lambda f=file: self.log(f"✅ Transferred: {f}", "success"))
                                
                                self.transferred_files += 1
                                
                            except Exception as e:
                                self.root.after(0, lambda f=file, e=str(e): self.log(f"❌ Failed {f}: {e}", "error"))
                            
                            processed += 1
                            progress = (processed / self.total_files) * 100
                            self.root.after(0, lambda p=progress: self.progress_var.set(p))
                            self.root.after(0, lambda s=f"Transferring... {processed}/{self.total_files}": self.status_var.set(s))
                
                self.root.after(0, self.transfer_complete)
                
            except Exception as e:
                self.root.after(0, lambda: self.log(f"❌ Fatal error: {str(e)}", "error"))
                self.root.after(0, lambda: self.transfer_btn.configure(state=tk.NORMAL))
                self.root.after(0, lambda: self.analyze_btn.configure(state=tk.NORMAL))
        
        threading.Thread(target=transfer_thread, daemon=True).start()
    
    def transfer_complete(self):
        self.status_var.set(f"Complete! Transferred {self.transferred_files} files")
        self.progress_var.set(100)
        self.log("🎉 Transfer completed successfully!", "success")
        messagebox.showinfo("Complete", f"Transfer finished!\n{self.transferred_files} files transferred successfully.")
        self.transfer_btn.configure(state=tk.NORMAL)
        self.analyze_btn.configure(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    app = FileTransferApp(root)
    root.mainloop()