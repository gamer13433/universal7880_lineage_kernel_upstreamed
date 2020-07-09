
int
nouveau_handle_new(struct nouveau_object *client, u32 _parent, u32 _handle,
		   u16 _oclass, void *data, u32 size,
		   struct nouveau_object **pobject)
{
	struct nouveau_object *parent = NULL;
	struct nouveau_object *engctx = NULL;
	struct nouveau_object *object = NULL;
	struct nouveau_object *engine;
	struct nouveau_oclass *oclass;
	struct nouveau_handle *handle;
	int ret;

	/* lookup parent object and ensure it *is* a parent */
	parent = nouveau_handle_ref(client, _parent);
	if (!parent) {
		nv_error(client, "parent 0x%08x not found\n", _parent);
		return -ENOENT;
	}

	if (!nv_iclass(parent, NV_PARENT_CLASS)) {
		nv_error(parent, "cannot have children\n");
		ret = -EINVAL;
		goto fail_class;
	}

	/* check that parent supports the requested subclass */
	ret = nouveau_parent_sclass(parent, _oclass, &engine, &oclass);
	if (ret) {
		nv_debug(parent, "illegal class 0x%04x\n", _oclass);
		goto fail_class;
	}

	/* make sure engine init has been completed *before* any objects
	 * it controls are created - the constructors may depend on
	 * state calculated at init (ie. default context construction)
	 */
	if (engine) {
		ret = nouveau_object_inc(engine);
		if (ret)
			goto fail_class;
	}

	/* if engine requires it, create a context object to insert
	 * between the parent and its children (eg. PGRAPH context)
	 */
	if (engine && nv_engine(engine)->cclass) {
		ret = nouveau_object_ctor(parent, engine,
					  nv_engine(engine)->cclass,
					  data, size, &engctx);
		if (ret)
			goto fail_engctx;
	} else {
		nouveau_object_ref(parent, &engctx);
	}

	/* finally, create new object and bind it to its handle */
	ret = nouveau_object_ctor(engctx, engine, oclass, data, size, &object);
	*pobject = object;
	if (ret)
		goto fail_ctor;

	ret = nouveau_object_inc(object);
	if (ret)
		goto fail_init;

	ret = nouveau_handle_create(parent, _parent, _handle, object, &handle);
	if (ret)
		goto fail_handle;

	ret = nouveau_handle_init(handle);
	if (ret)
		nouveau_handle_destroy(handle);

fail_handle:
	nouveau_object_dec(object, false);
fail_init:
	nouveau_object_ref(NULL, &object);
fail_ctor:
	nouveau_object_ref(NULL, &engctx);
fail_engctx:
	if (engine)
		nouveau_object_dec(engine, false);
fail_class:
	nouveau_object_ref(NULL, &parent);
	return ret;
}

int
nouveau_handle_del(struct nouveau_object *client, u32 _parent, u32 _handle)
{
	struct nouveau_object *parent = NULL;
	struct nouveau_object *namedb = NULL;
	struct nouveau_handle *handle = NULL;

	parent = nouveau_handle_ref(client, _parent);
	if (!parent)
		return -ENOENT;

	namedb = nv_pclass(parent, NV_NAMEDB_CLASS);
	if (namedb) {
		handle = nouveau_namedb_get(nv_namedb(namedb), _handle);
		if (handle) {
			nouveau_namedb_put(handle);
			nouveau_handle_fini(handle, false);
			nouveau_handle_destroy(handle);
		}
	}

	nouveau_object_ref(NULL, &parent);
	return handle ? 0 : -EINVAL;
}
