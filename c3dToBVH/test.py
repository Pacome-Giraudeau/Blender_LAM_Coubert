class CustomFileDialog(filedialog.FileDialog):
    def __init__(self, master, title=None):
        super().__init__(master, title)
        
    def ok(self, event=None):
        # Personnalisation de l'action lorsqu'on clique sur "OK"
        print("Fichier sélectionné :", self.get_selection())
        super().ok(event)
    
    def get_selection(self):
        # Retourne le fichier sélectionné
        return self.selection

    def apply(self):
        # Logique supplémentaire après la sélection
        print("Appel à la méthode apply() pour gérer le fichier sélectionné.")
        super().apply()

if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()  # On masque la fenêtre principale
    
    dialog = CustomFileDialog(root, title="Sélectionnez un fichier")
    dialog.show()  # Affiche la boîte de dialogue personnalisée