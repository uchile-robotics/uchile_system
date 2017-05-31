# Hooks para repositorios git

Durante el proceso de instalación del workspace de UChile ROS Framework, se asignan *precommit hooks* para todos los repositorios de *base_ws*, *soft_ws* y *high_ws*, sumado a *uchile_system*.

Los hooks instalados son sólo del tipo *precommit*, es decir, se corren al momento que el usuario intenta hacer un *commit* y tienen la facultad de permitirlo o denegarlo. Más información sobre los hooks en la [documentación oficial](https://git-scm.com/docs/githooks).

Los hooks están implementados mediantes scripts de *sh*, en el directorio `${UCHILE_SYSTEM}/hooks`. 

## Instalación y desinstalación

El punto de acceso de git a los hooks es el archivo `${UCHILE_SYSTEM}/hooks/precommit`, el que es copiado en la carpeta `.git/hooks/` del repositorio de interés, durante el proceso de instalación.

Para habilitar los hooks en un repositorio, basta copiar el archivo `precommit` en el repositorio que sea de interés. Además, paa funcionar requieren que la variable de entorno `$GITHOOKS_PATH` esté correctamente seteada en el terminal. Más información en el [mismo archivo](https://github.com/uchile-robotics/uchile_system/blob/develop/hooks/pre-commit).

Para deshabilitar el hook de un repositorio, basta eliminar el archivo `.git/hooks/precommit`.


## Hooks

Los hooks chequean distintos aspectos de calidad del código que va a ser agregado/modificado en el commit. Los hooks **no chequean archivos a ser eliminados, ni archivos no considerados en el commit**.

En orden, los chequeos actuales son:

1. Nombres de archivo válidos: Sólo se aceptan los caracteres que pasen en la expresión regular: `[:alnum:][/._\-]`.
2. Tamaño máximo de los archivos: Se aceptan archivos de **hasta 400kB.**
3. Revisar que archivos no contengan signos de merge conflict, es decir, líneas del estilo: `<<<<<<<` o `>>>>>>>`.
4. Linter para archivos **.sh**
5. Linter para archivos **.bash**
6. Linter para archivos **python**
7. Linter para archivos **XML**
8. Linter para archivos **YAML**
9. Linter para archivos **C++**

Los archivos redireccionados a cada subhook son seleccionados según su extensión. Archivos sin extensión no serán chequeados.


