# Troubleshooting

* [Actualizar la instalación](#actualización-de-la-instalación)
* [Mi consola no conoce los comandos del framework](#mi-consola-no-conoce-los-comandos-del-framework)
* [Arreglando problemas del workspace](#arreglando-problemas-del-workspace)

## Actualización de la instalación

En caso de querer descargar repositorios faltantes o actualizar la instalación, siempre se puede correr el instalador nuevamente y sin miedo a romper lo que ya está funcionando. El único efecto adverso es que algunos repositorios queden en su rama o commit default (para el caso de los forks).

Observaciones sobre el instalador:

* Ejecutarlo **toma poco tiempo**, a menos haya que descargar repositorios. 
* Debiera corregir workspaces faltantes o mal configurados.
* El instalador descarga repositorios faltantes, instala hooks de git y crea/corrige links simbólicos.


Debes correr el instalador nuevamente desde un terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>):

```bash
# crear backup de uchile.sh, que será reemplazado
cp "$HOME"/uchile.sh{,.bkp}

# correr instalador
bash "$HOME"/uchile_ws/system/install/ws_installer.bash
```

Recuerda actualizar tus configuraciones del archivo `uchile.sh`, pues tras actualizar contendrá las por defecto. Ojo que las antiguas quedan almacenadas en `uchile.sh.bkp`.


## Mi consola no conoce los comandos del framework

Probablemente la configuración de la consola no es correcta!. Debes revisar que el archivo de configuración de tu consola favorita (`.bashrc` o `.zshrc`) tenga las líneas correctas y que no le sobren otras líneas. Más información en la [sección correspondiente de la instalación](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/installation.md#habilitar-workspace-para-uso-en-consola).


## Arreglando problemas del workspace

### uchile_fix_links

Ver: `uchile_fix_links -h`

### uchile_clean_workspace

Ver: `uchile_clean_workspace -h`

